/**
 * nodes-io-service.js
 * Gathers all ROS2 nodes with the topics they publish (O) and subscribe to (I).
 * Provides a table view + one-click CSV download.
 */

class NodesIOService {
    constructor() {
        // Holds gathered rows in the form { node, topic, dir } where dir = 'I' | 'O'
        this.nodesIoRows = [];
        this.pendingNodes = new Set();
        this.simulation = null;
        this.initialized = false;
        // Store node colors to ensure consistency between renders
        this.nodeColorMap = new Map();
        this.nextColorIndex = 0;
        
        // Set default value for showing unsubscribed topics (false = off by default)
        if (window.showUnsubscribedArrows === undefined) {
            window.showUnsubscribedArrows = false;
        }
        
        // Default configuration
        this.config = {
            skipNodes: ['/rosapi', '/rosapi_params'],
            skipTopics: ['/rosout', '/parameter_events'],
            graphHeight: 600,
            nodeRadius: 12
        };
    }

    /**
     * Initialize the service by setting up event listeners and ensuring UI elements exist
     */
    init() {
        if (this.initialized) return;
        
        this.ensureTable();
        this.ensureGraphContainer();
        this.attachEventListeners();
        
        // Initialize immediately if ROS is already connected
        if (rosConnection && rosConnection.readyState === WebSocket.OPEN) {
            this.refresh();
        }
        
        this.log('Nodes-IO service ready');
        this.initialized = true;
    }

    /**
     * Create table element if it doesn't exist
     */
    ensureTable() {
        if (document.getElementById('nodes-io-table')) return;
        
        const nodesBox = document.querySelector('#nodes-io-section .box');        if (!nodesBox) return;
        
        const table = document.createElement('table');
        table.id = 'nodes-io-table';
        table.className = 'table is-fullwidth is-striped is-hoverable mt-4';
        nodesBox.appendChild(table);
    }

    /**
     * Create graph container element if it doesn't exist
     */
    ensureGraphContainer() {
        const box = document.querySelector('#nodes-io-section .box');
        if (!box || document.getElementById('nodes-io-graph')) return;
        
        const graphDiv = document.createElement('div');
        graphDiv.id = 'nodes-io-graph';
        graphDiv.style.height = `${this.config.graphHeight}px`;
        graphDiv.style.border = '1px solid #ddd';
        box.appendChild(graphDiv);
    }

    /**
     * Attach all event listeners
     */
    attachEventListeners() {
        // ROS connection events
        document.addEventListener('ros-connected', () => this.refresh());
        document.addEventListener('nodes-list-received', e => this.handleNodesList(e.detail));
        document.addEventListener('node-details-received', e => this.handleNodeDetails(e.detail));
        
        // UI interaction events
        this.attachButtonListeners();
        
        // Watch for dynamically added buttons
        const observer = new MutationObserver(() => this.attachButtonListeners());
        observer.observe(document.body, { childList: true, subtree: true });
    }

    /**
     * Attach event listeners to UI buttons
     */
    attachButtonListeners() {
        // Download button
        const downloadBtn = document.getElementById('nodes-io-download');
        if (downloadBtn && !downloadBtn._listenerAttached) {
            downloadBtn.addEventListener('click', () => this.downloadCsv());
            downloadBtn._listenerAttached = true;
        }

        // Refresh button
        const showBtn = document.getElementById('nodes-io-show');
        if (showBtn && !showBtn._listenerAttached) {
            showBtn.addEventListener('click', () => {
                this.ensureTable();
                this.refresh();
            });
            showBtn._listenerAttached = true;
        }
    }

    /**
     * Refresh nodes and topics data
     */
    refresh() {
        this.ensureTable();
        if (!rosConnection || rosConnection.readyState !== WebSocket.OPEN) {
            this.log('WebSocket not open – cannot request nodes list', 'warn');
            return;
        }
        
        this.nodesIoRows = [];
        this.pendingNodes.clear();
        this.renderTable(); // Empty while loading
        
        // Request nodes list from ROS
        const reqId = 'get_nodes_io_' + Date.now();
        const sent = callRosService('/rosapi/nodes', {}, reqId);
        this.log(`Requested nodes list (id=${reqId}) – sent=${sent}`);
    }

    /**
     * Handle the nodes list response from ROS
     */
    handleNodesList(msg) {
        console.log('[Nodes-IO] Raw nodes list msg', msg);
        let nodes = [];
        if (!msg || !msg.values) {
            this.log('Invalid nodes list response format', 'error');
        } else if (Array.isArray(msg.values.nodes)) {
            nodes = msg.values.nodes;
        } else if (Array.isArray(msg.values.names)) {
            nodes = msg.values.names;
        } else if (msg.values.result && Array.isArray(msg.values.result.names)) {
            nodes = msg.values.result.names;
        } else if (msg.values.result && Array.isArray(msg.values.result.nodes)) {
            nodes = msg.values.result.nodes;
        }

        // Fallback: some bridges wrap nodes array directly under msg.values (ROS1 style)
        if (nodes.length === 0 && Array.isArray(msg.values)) {
            nodes = msg.values;
        }

        this.log(`Parsed nodes list – count=${nodes.length}`);
        nodes.forEach(n => {
            this.pendingNodes.add(n);
            const reqId = 'get_node_details_io_' + n + '_' + Date.now();
            callRosService('/rosapi/node_details', { node: n }, reqId);
        });
        
        if (nodes.length === 0) this.log('No nodes found', 'warn');
    }

    /**
     * Handle node details response from ROS
     */
    handleNodeDetails(msg) {
        // Node name may not be included in the response for some bridges.
        // Primary sources:
        //   • msg.values.node (ROS1 style)
        //   • msg.values.result.node (some ROS2 bridges)
        // Fallback: parse it from the request ID we generated earlier.
        let nodeName = msg.values.node || (msg.values.result && msg.values.result.node);
        if (!nodeName && msg.id && msg.id.startsWith('get_node_details_io_')) {
            // ID pattern: get_node_details_io_<node>_<timestamp>
            const parts = msg.id.split('_');
            if (parts.length >= 5) { // expect at least: [get,node,details,io,<node>,<ts>]
                // Rejoin everything between 'io' and the last part (timestamp) to support underscores in node names.
                nodeName = parts.slice(4, -1).join('_');
                if (!nodeName.startsWith('/')) nodeName = '/' + nodeName; // Normalise leading slash
            }
        }

        if (!nodeName) return;
        this.clearPending(nodeName);

        // Account for different rosapi implementations:
        //  - ROS1 style: pubs / subs
        //  - Older ROS2 bridges: publishers / subscribers
        //  - Current ROS2 bridges: publications / subscriptions
        //  - Foxglove bridge: values.result.*
        const result = msg.values.result || {};
        const pubsRaw =
            msg.values.pubs ||
            msg.values.publishers ||
            msg.values.publications ||
            msg.values.publishing ||
            result.publishers ||
            result.publications ||
            result.publishing;

        const subsRaw =
            msg.values.subs ||
            msg.values.subscribers ||
            msg.values.subscriptions ||
            msg.values.subscribing ||
            result.subscribers ||
            result.subscriptions ||
            result.subscribing;

        const pubs = this.extractTopicNames(pubsRaw);
        const subs = this.extractTopicNames(subsRaw);

        pubs.forEach(t => this.nodesIoRows.push({ node: nodeName, topic: t, dir: 'O' }));
        subs.forEach(t => this.nodesIoRows.push({ node: nodeName, topic: t, dir: 'I' }));

        console.debug(`[Nodes-IO] Node ${nodeName}: pubs=${pubs.length}, subs=${subs.length}, pending=${this.pendingNodes.size}`);

        if (this.pendingNodes.size === 0) {
            this.log(`Collected IO info for all nodes – total rows: ${this.nodesIoRows.length}`);
            console.log('[Nodes-IO] Final rows array', this.nodesIoRows);
        }
        // Render incrementally so the user sees data even while some nodes are still pending
        this.renderTable();
    }

    /**
     * Extract topic names from various ROS bridge response shapes
     */
    extractTopicNames(arr) {
        if (!Array.isArray(arr)) return [];
        return arr.map(item => {
            if (typeof item === 'string') return item;
            if (item && item.name) return item.name;
            return null;
        }).filter(Boolean);
    }

    /**
     * Clear a node from pending regardless of leading slash mismatch
     */
    clearPending(nodeName) {
        if (this.pendingNodes.has(nodeName)) {
            this.pendingNodes.delete(nodeName);
            return;
        }
        
        // Handle potential leading slash mismatch
        const withSlash = nodeName.startsWith('/') ? nodeName : '/' + nodeName;
        const withoutSlash = nodeName.startsWith('/') ? nodeName.substring(1) : nodeName;
        
        this.pendingNodes.delete(withSlash);
        this.pendingNodes.delete(withoutSlash);
    }

    /**
     * Log messages with appropriate prefix
     */
    log(msg, level = 'info') {
        if (typeof logToConsole === 'function') {
            logToConsole('[Nodes-IO] ' + msg, level);
        } else {
            console[level](msg);
        }
    }

    /**
     * Render the table in #nodes-io-table
     */
    renderTable() {
        this.ensureTable();
        const tbl = document.getElementById('nodes-io-table');
        if (!tbl) return; // Not added to DOM yet

        const SKIP_NODES = this.config.skipNodes;
        const SKIP_TOPICS = this.config.skipTopics;

        // Deduplicate rows accurately (node+topic+dir unique)
        const allUnique = this.nodesIoRows.filter((v,i,a)=>a.findIndex(e=>e.node===v.node&&e.topic===v.topic&&e.dir===e.dir)===i);

        // Rows we show: only publications (dir==='O') excluding skipped
        const filteredRows = allUnique.filter(r => r.dir === 'O' && !SKIP_NODES.includes(r.node) && !SKIP_TOPICS.includes(r.topic));

        // Sort by node, then topic
        filteredRows.sort((a,b)=>{
            if (a.node === b.node) return a.topic.localeCompare(b.topic);
            return a.node.localeCompare(b.node);
        });

        // Build HTML
        let html = '<thead><tr><th>Publisher</th><th>Topic</th><th>Subscriber</th></tr></thead><tbody>';
        if (filteredRows.length === 0) {
            html += '<tr><td colspan="3" class="has-text-centered">Loading or no data</td></tr>';
        } else {
            filteredRows.forEach(r => {
                // Find peer nodes on the same topic (exclude self)
                const peers = allUnique
                    .filter(p => p.topic === r.topic && p.node !== r.node && p.dir !== r.dir && !SKIP_NODES.includes(p.node) && !SKIP_TOPICS.includes(p.topic))
                    .map(p => p.node)
                    .filter((v, i, a) => a.indexOf(v) === i);

                if (peers.length === 0) {
                    html += `<tr><td><code>${r.node}</code></td><td><code>${r.topic}</code></td><td></td></tr>`;
                } else {
                    peers.forEach(peer => {
                        html += `<tr><td><code>${r.node}</code></td><td><code>${r.topic}</code></td><td>${peer}</td></tr>`;
                    });
                }
            });
        }
        html += '</tbody>';
        tbl.innerHTML = html;
        console.debug(`[Nodes-IO] renderTable – rows=${this.nodesIoRows.length}`);

        // Also update network graph if d3 is available
        if (typeof d3 !== 'undefined') {
            try {
                this.renderGraph(filteredRows, allUnique);
            } catch (e) {
                console.error('[Nodes-IO] renderGraph error', e);
            }
        } else {
            if (!window.__d3MissingLogged) {
                console.warn('[Nodes-IO] d3 library not found – graph disabled');
                window.__d3MissingLogged = true;
            }
        }
    }

    /**
     * Render the graph
     */
    renderGraph(displayRows, allRows) {
        const container = document.getElementById('nodes-io-graph');
        if (!container) return;

        // Store a reference to the service for use inside callbacks
        const self = this;

        // Some browsers report 0 width if the section is still hidden (e.g., first time tab becomes visible).
        // Defer rendering briefly until the element has a measurable width to ensure text bounding boxes are correct.
        const measuredWidth = container.getBoundingClientRect().width;
        if (measuredWidth === 0) {
            // Try again on next animation frame; bail out now.
            requestAnimationFrame(() => this.renderGraph(displayRows, allRows));
            return;
        }
        const width = measuredWidth || 800;
        const height = container.clientHeight || 600;
        const nodeRadius = this.config.nodeRadius;

        // Clear previous SVG
        container.innerHTML = '';

        const svg = d3.select(container)
            .append('svg')
            .attr('width', width)
            .attr('height', height)
            .attr('viewBox', [0, 0, width, height])
            .call(
                d3.zoom().scaleExtent([0.5, 5]).on('zoom', (event) => {
                    main.attr('transform', event.transform);
                })
            );

        // Re-create overlay checkbox each render (cleared by innerHTML)
        if (!container.querySelector('#toggle-unsub-arrows')) {
            const overlay = document.createElement('label');
            overlay.className = 'checkbox';
            overlay.style.position = 'absolute';
            overlay.style.top = '6px';
            overlay.style.left = '6px';
            overlay.style.background = 'rgba(255,255,255,0.8)';
            overlay.style.padding = '2px 4px';
            overlay.style.borderRadius = '4px';
            const cb = document.createElement('input');
            cb.type = 'checkbox';
            cb.id = 'toggle-unsub-arrows';
            // Use the global flag directly
            cb.checked = window.showUnsubscribedArrows;
            overlay.appendChild(cb);
            overlay.append(' Show un-subscribed topics');
            container.style.position = 'relative';
            container.appendChild(overlay);
            cb.addEventListener('change', () => {
                // Store the value before re-rendering
                window.showUnsubscribedArrows = cb.checked;
                self.renderTable();
            });
        } else {
            // Update existing checkbox to match the current state
            const cb = container.querySelector('#toggle-unsub-arrows');
            cb.checked = window.showUnsubscribedArrows;
        }

        const main = svg.append('g');

        // Global boolean flag to enable arrows for unsubscribed topics (dummy targets)
        const INCLUDE_DUMMY = window.showUnsubscribedArrows;

        // Build links: aggregate by source→target so we can show multiple topics as thicker arrows
        const SKIP_NODES_G = this.config.skipNodes;
        const dummyNodes = new Set(); // holds ids of invisible endpoints
        const linkMap = new Map(); // key: "src->dst" value: {source,target,topics[]}
        let dummyCounter = 0; // counter to ensure unique dummy IDs
        displayRows.forEach(pub => {
            if (SKIP_NODES_G.includes(pub.node)) return; // skip
            const subs = allRows.filter(r => r.topic === pub.topic && r.dir !== pub.dir && !SKIP_NODES_G.includes(r.node));
            subs.forEach(sub => {
                const key = `${pub.node}->${sub.node}`;
                if (!linkMap.has(key)) {
                    linkMap.set(key, { source: pub.node, target: sub.node, topics: [] });
                }
                linkMap.get(key).topics.push(pub.topic);
            });
            if (INCLUDE_DUMMY && subs.length === 0) {
                // Create a unique dummy ID for every missing subscriber so that
                // multiple publishers to the same topic do not share one dummy node.
                const dummyId = `__dummy__${dummyCounter++}`;
                dummyNodes.add(dummyId);
                const key = `${pub.node}->${dummyId}`;
                if (!linkMap.has(key)) {
                    linkMap.set(key, { source: pub.node, target: dummyId, topics: [] });
                }
                linkMap.get(key).topics.push(pub.topic);
            }
        });

        const links = Array.from(linkMap.values()).map(l => ({ ...l, count: l.topics.length }));

        // Identify dummy nodes (unsubscribed topics) **before rendering**
        const dummySet = new Set([...dummyNodes]); // set of ids for dummy (invisible) endpoints

        // Collect every node including dummy endpoints
        const nodeIds = new Set([...allRows.map(r => r.node).filter(n => !SKIP_NODES_G.includes(n)), ...(INCLUDE_DUMMY ? dummyNodes : [])]);
        const nodes = Array.from(nodeIds, id => ({ id, dummy: dummyNodes.has(id) }));
        const nodeById = new Map(nodes.map(d => [d.id, d]));
        links.forEach(l => { 
            l.source = nodeById.get(l.source); 
            l.target = nodeById.get(l.target); 
        });

        // Get a consistent color for a node
        const getNodeColor = this.getNodeColor.bind(this);

        // Unique colour per node (avoid reuse)
        const basePalette = [
            '#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', // Tableau
            '#8c564b', '#e377c2', '#ff1493', '#bcbd22', '#17becf', // Tableau
            '#3366cc', '#dc3912', '#ff9900', '#109618', '#990099', // Google
            '#0099c6', '#dd4477', '#66aa00', '#b82e2e', '#316395', // Google
            '#994499', '#22aa99', '#aaaa11', '#6633cc', '#e67300'  // Google
        ];
        
        const defs = svg.append('defs');
        const markerCache = new Map();
        
        function getArrowId(col) {
            if (!markerCache.has(col)) {
                const clean = col.replace('#', '');
                const id = `arrow-${clean}`;
                const mk = defs.append('marker')
                    .attr('id', id)
                    .attr('viewBox', '0 -6 12 12')
                    .attr('refX', 6)
                    .attr('refY', 0)
                    .attr('markerWidth', 10)
                    .attr('markerHeight', 10)
                    .attr('orient', 'auto');
                mk.append('path')
                    .attr('d', 'M0,-6L12,0L0,6')
                    .attr('fill', col)
                    .attr('stroke', col);
                markerCache.set(col, id);
            }
            return `url(#${markerCache.get(col)})`;
        }

        // ---- Orphan detection & colour helper ----
        // Build degree maps (exclude dummy)
        const inDeg = new Map();
        const outDeg = new Map();
        links.forEach(l => {
            const sId = l.source.id ?? l.source; // handle object vs id
            const tId = l.target.id ?? l.target;
            const srcIsReal = !dummySet.has(sId);
            const tgtIsReal = !dummySet.has(tId);
            // Count degrees only when connected to a REAL node
            if (srcIsReal && tgtIsReal) {
                outDeg.set(sId, (outDeg.get(sId) || 0) + 1);
                inDeg.set(tId, (inDeg.get(tId) || 0) + 1);
            } else if (srcIsReal && !tgtIsReal) {
                // source connects only to dummy – do NOT increment outDeg so it can be orphan
                // no-op
            } else if (!srcIsReal && tgtIsReal) {
                // link from dummy to real – shouldn't happen, but safeguard
                inDeg.set(tId, (inDeg.get(tId) || 0) + 0);
            }
        });

        // Orphans: nodes with no incoming or outgoing real links (degree 0)
        const orphanSet = new Set();
        nodes.forEach(n => {
            if (!n.dummy && !inDeg.has(n.id) && !outDeg.has(n.id)) {
                orphanSet.add(n.id);
            }
        });

        // Grey-out orphan nodes regardless of toggle state
        const visualColor = id => (orphanSet.has(id) ? '#999' : getNodeColor(id));

        const link = main.append('g')
            .attr('stroke-width', 1.5) // constant width even for multi-topic
            .attr('stroke-opacity', 0.8)
            .attr('fill', 'none')
          .selectAll('path')
          .data(links)
          .join('path')
            .attr('stroke', d => d.target.dummy ? '#999' : visualColor(d.source.id))
            .attr('marker-end', d => getArrowId(d.target.dummy ? '#999' : visualColor(d.source.id)));

        // Annotations for link topics using d3-svg-annotation
        const annotationsData = links.map(l => ({
            note: { label: l.topics.join('\n') },
            x: 0,
            y: 0,
            dx: 0,
            dy: -12,
            color: (l.target.dummy ? '#999' : visualColor(l.source.id))
        }));

        const annotationGen = d3.annotation()
            .type(d3.annotationLabel)
            .annotations(annotationsData)
            .accessors({ x: d => d.x, y: d => d.y })
            .accessorsInverse({ x: d => d.x, y: d => d.y });

        const annotationGroup = main.append('g').attr('class', 'link-annotations').call(annotationGen);

        function formatAnnotationLabels(group){
            group.selectAll('.annotation-note-label')
                .style('font-size', '8px')
                .each(function(){
                    const txt = this.textContent;
                    if(txt.includes('\n')){
                        const parts = txt.split('\n');
                        this.textContent='';
                        parts.forEach((p,i)=>{
                            const t = document.createElementNS('http://www.w3.org/2000/svg','tspan');
                            t.setAttribute('x','0');
                            if(i>0) t.setAttribute('dy','1em');
                            t.textContent=p;
                            this.appendChild(t);
                        });
                    }
                });
        }
        
        formatAnnotationLabels(annotationGroup);

        const node = main.append('g')
            .attr('stroke', '#fff')
            .attr('stroke-width', 1.5)
          .selectAll('circle')
          .data(nodes.filter(d => !d.dummy))
          .join('circle')
            .attr('r', nodeRadius)
            .attr('fill', d => visualColor(d.id))
            .call(dragBehaviour());

        // Node labels with colored background and white text
        const nodeLabel = main.append('g')
          .selectAll('g')
          .data(nodes.filter(d => !d.dummy))
          .join('g')
            .attr('pointer-events', 'none');

        // Build label sub-elements once per node (rect + text)
        nodeLabel.each(function(d, i) {
            const g = d3.select(this);
            if (g.select('text').empty()) {
                const t = g.append('text')
                    .attr('fill', '#fff')
                    .attr('font-size', 10)
                    .text(d.id.replace(/^\//, ''));
                // Use both getBBox and getComputedTextLength for robustness
                let bb = t.node().getBBox();
                // Some browsers report 0 width immediately after insert; guard against that
                if (bb.width < 2) {
                    const w = t.node().getComputedTextLength();
                    bb = { x: 0, y: -7, width: w, height: 12 }; // approximate height
                }
                let rect = g.select('rect');
                if (rect.empty()) rect = g.insert('rect', 'text');
                rect
                    .attr('x', bb.x - 4)
                    .attr('y', bb.y - 2)
                    .attr('width', bb.width + 8)
                    .attr('height', bb.height + 4)
                    .attr('rx', 3)
                    .attr('ry', 3)
                    .attr('fill', visualColor(d.id));
            }
        });

        // Save simulation reference in the class instance
        this.simulation = d3.forceSimulation(nodes)
            .force('link', d3.forceLink(links).id(d => d.id).distance(100))
            // Adjust orphan positioning based on toggle state - reduce distance further
            .force('orphan', d3.forceRadial(
                // When toggle is on (showing unsubscribed topics), keep orphans very close to center
                window.showUnsubscribedArrows ? Math.min(width, height) / 20 : Math.min(width, height) / 3.5,
                width / 2,
                height / 2
            ).strength(d => orphanSet.has(d.id) ? 0.5 : 0))
            // Add separate force for dummy nodes (unsubscribed topics)
            .force('dummy', d3.forceRadial(
                // Keep dummy nodes close to their connected nodes
                Math.min(width, height) / 100,
                width / 2,
                height / 2
            ).strength(d => dummySet.has(d.id) ? 0.04 : 0))
            // Adjust charge force to treat regular nodes differently from dummy nodes
            .force('charge', d3.forceManyBody().strength(d => dummySet.has(d.id) ? -20 : -200))
            .force('center', d3.forceCenter(width / 2, height / 2))
            // Reduce collision radius for dummy nodes
            .force('collide', d3.forceCollide(d => dummySet.has(d.id) ? nodeRadius/2 : nodeRadius))
            .on('tick', tick);

        // Tick function defined separately with captured scope
        function tick() {
            link.attr('d', d => {
                const dx = d.target.x - d.source.x;
                const dy = d.target.y - d.source.y;
                const len = Math.hypot(dx, dy);
                if (len === 0) return '';
                
                return `M${d.source.x},${d.source.y}L${d.target.x - (dx/len)*(nodeRadius+11)},${d.target.y - (dy/len)*(nodeRadius+6)}`;
            });
            
            annotationsData.forEach((a,i) => {
                const l = links[i];
                if (l && l.source && l.target) {
                    a.x = (l.source.x + l.target.x)/2;
                    a.y = (l.source.y + l.target.y)/2;
                }
            });
            
            annotationGen.annotations(annotationsData);
            annotationGroup.call(annotationGen);
            formatAnnotationLabels(annotationGroup);

            node.attr('cx', d => d.x).attr('cy', d => d.y);
            nodeLabel.attr('transform', d => `translate(${d.x + 14},${d.y})`);
        }

        function dragBehaviour() {
            return d3.drag()
                .on('start', (event, d) => {
                    if (!event.active) self.simulation.alphaTarget(0.3).restart();
                    d.fx = d.x;
                    d.fy = d.y;
                })
                .on('drag', (event, d) => {
                    d.fx = event.x;
                    d.fy = event.y;
                })
                .on('end', (event, d) => {
                    if (!event.active) self.simulation.alphaTarget(0);
                    d.fx = null;
                    d.fy = null;
                });
        }
    }

    /**
     * Get a consistent color for a node
     * @param {string} nodeId - The node ID
     * @returns {string} - The color
     */
    getNodeColor(nodeId) {
        if (!this.nodeColorMap.has(nodeId)) {
            // Use a custom color palette with more vibrant, higher contrast colors
            // Avoid very light colors that are hard to see
            const vibrantPalette = [
                '#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', // Tableau
                '#8c564b', '#e377c2', '#ff1493', '#bcbd22', '#17becf', // Tableau
                '#3366cc', '#dc3912', '#ff9900', '#109618', '#990099', // Google
                '#0099c6', '#dd4477', '#66aa00', '#b82e2e', '#316395', // Google
                '#994499', '#22aa99', '#aaaa11', '#6633cc', '#e67300'  // Google
            ];
            
            if (this.nextColorIndex < vibrantPalette.length) {
                this.nodeColorMap.set(nodeId, vibrantPalette[this.nextColorIndex++]);
            } else {
                // Generate additional colors via HSL with controlled lightness
                // Use HSL to ensure we don't get colors that are too light
                const hue = (this.nextColorIndex * 137.5) % 360; // Golden angle approximation
                const saturation = 75 + (this.nextColorIndex % 25); // 75-100%
                const lightness = 35 + (this.nextColorIndex % 30); // 35-65%, avoid very light colors
                this.nextColorIndex++;
                this.nodeColorMap.set(nodeId, `hsl(${hue}, ${saturation}%, ${lightness}%)`);
            }
        }
        return this.nodeColorMap.get(nodeId);
    }

    /**
     * Generates CSV and triggers browser download
     */
    downloadCsv() {
        if (this.nodesIoRows.length === 0) {
            alert('No data to download yet.');
            return;
        }
        let csv = 'publisher,topic,subscriber\n';
        const uniqueRows = this.nodesIoRows.filter((v, i, a) =>
            a.findIndex(e => e.node === v.node && e.topic === v.topic && e.dir === e.dir) === i);
        const SKIP_NODES = this.config.skipNodes;
        const SKIP_TOPICS = this.config.skipTopics;
        const filteredRows = uniqueRows.filter(r => r.dir === 'O' && !SKIP_NODES.includes(r.node) && !SKIP_TOPICS.includes(r.topic));
        filteredRows.forEach(r => {
            const peers = uniqueRows
                .filter(p => p.topic === r.topic && p.node !== r.node && p.dir !== r.dir && !SKIP_NODES.includes(p.node) && !SKIP_TOPICS.includes(p.topic))
                .map(p => p.node)
                .filter((v, i, a) => a.indexOf(v) === i);

            if (peers.length === 0) {
                csv += `${r.node},${r.topic},\n`;
            } else {
                peers.forEach(peer => {
                    csv += `${r.node},${r.topic},${peer}\n`;
                });
            }
        });
        const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        const ts = new Date().toISOString().replace(/[:.]/g, '-');
        a.download = `nodes_io_${ts}.csv`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    }
}

// Create singleton instance
const nodesIoService = new NodesIOService();

// Global functions for backwards compatibility
function ensureTable() {
    nodesIoService.ensureTable();
}

function initNodesIoService() {
    nodesIoService.init();
}

function refreshNodesIo() {
    nodesIoService.refresh();
}

function handleNodesList(msg) {
    nodesIoService.handleNodesList(msg);
}

function handleNodeDetails(msg) {
    nodesIoService.handleNodeDetails(msg);
}

function clearPending(nodeName) {
    nodesIoService.clearPending(nodeName);
}

function renderTable() {
    nodesIoService.renderTable();
}

function renderGraph(displayRows, allRows) {
    nodesIoService.renderGraph(displayRows, allRows);
}

function downloadCsv() {
    nodesIoService.downloadCsv();
}

function log(msg, level = 'info') {
    nodesIoService.log(msg, level);
}

// Kick-off when DOM is ready. If the DOM is already loaded (script injected late by
// the sequential loader), run immediately; otherwise defer until DOMContentLoaded.
function bootstrapNodesIo() {
    if (!window.__nodesIoServiceInitialized) {
        initNodesIoService();
        window.__nodesIoServiceInitialized = true;
    }
}

if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', bootstrapNodesIo);
} else {
    // DOMContentLoaded already fired – initialise now.
    bootstrapNodesIo();
}
