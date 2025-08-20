// diagnostics-service.js
// Subscribe to /diagnostics and show compact table in diagnostics-section

(function(){
    let subscribed = false;
    let diagMap = new Map(); // key: status.name, value: latest DiagnosticStatus (deduplicated by name)
    let subscriptionId = null;
    let diagHandler = null;
    // Track which diagnostic names are currently expanded so the state survives re-renders
    const openDetails = new Set();

    // Inject small utility styles once for table formatting
    const STYLE_ID = 'diagnostics-style';
    if(!document.getElementById(STYLE_ID)){
        const style = document.createElement('style');
        style.id = STYLE_ID;
        style.textContent = `
            /* Name column 10 characters (~80px) wider */
            .diag-name{max-width:240px; white-space:nowrap; overflow:hidden; text-overflow:ellipsis;}
            /* Truncate lengthy message column too */
            .diag-msg{max-width:300px; white-space:nowrap; overflow:hidden; text-overflow:ellipsis;}
            /* Arrow icon */
            .toggle-arrow{cursor:pointer; user-select:none; width:1em; display:inline-block; text-align:center;}
        `;
        document.head.appendChild(style);
    }

    function levelToClass(level){
        switch(level){
            case 0: return 'has-text-success-dark'; // OK
            case 1: return 'has-text-warning-dark'; // WARN
            case 2: return 'has-text-danger';  // ERROR/STALE
            default: return '';
        }
    }

    function levelToText(level){
        switch(level){
            case 0: return 'OK';
            case 1: return 'WARN';
            case 2: return 'ERROR';
            default: return level;
        }
    }

    function renderTable(){
        const container = document.getElementById('diagnostics-table-container');
        const placeholder = document.getElementById('diagnostics-placeholder');
        if(!container) return;

        // Sort diagnostic statuses alphabetically (A→Z) by their name for consistent ordering
        // Build a summary row plus a hidden details row for each status
        const rows = Array.from(diagMap.values())
            .sort((a,b)=>a.name.localeCompare(b.name))
            .map(stat=>{
                const lvlClass = levelToClass(stat.level);
                const levelText = levelToText(stat.level);
                const timeStr = stat.receivedAt ? new Date(stat.receivedAt).toLocaleTimeString() : '-';
                const shortMsg = stat.message.length>60 ? stat.message.slice(0,60)+'…' : stat.message;

                // Build details list (hardware id + key/values)
                const kvList = (stat.values || []).map(kv=>`<li><strong>${kv.key}</strong>: ${kv.value}</li>`).join('');
                const detailsVisible = openDetails.has(stat.name);
                const detailsHtml = `<tr class="diag-details" style="display:${detailsVisible ? 'table-row':'none'};"><td colspan="4">
                        <div><strong>Hardware ID:</strong> ${stat.hardware_id || '-'}</div>
                        ${kvList ? `<ul>${kvList}</ul>` : ''}
                    </td></tr>`;

                // Summary row is clickable (pointer cursor) to toggle the following details row
                const arrowSymbol = detailsVisible ? '▲' : '▼';
                const summaryHtml = `<tr class="diag-summary ${lvlClass}${detailsVisible ? ' is-open':''}" data-name="${stat.name}">
                        <td class="diag-name" title="${stat.name}"><span class="toggle-arrow">${arrowSymbol}</span> ${stat.name}</td>
                        <td class="diag-msg" title="${stat.message}">${shortMsg}</td>
                        <td>${levelText}</td>
                        <td>${timeStr}</td>
                    </tr>`;
                return summaryHtml + detailsHtml;
            })
            .join('');

        // Render table skeleton with rows (possibly empty)
        container.innerHTML = `
        <table class="table is-fullwidth is-striped is-hoverable is-narrow">
            <thead><tr><th class="diag-name">Name</th><th class="diag-msg">Message</th><th>Level</th><th>Updated</th></tr></thead>
            <tbody>${rows || '<tr><td colspan="4" class="has-text-centered">No diagnostics received yet</td></tr>'}</tbody>
        </table>`;

        // Show or hide placeholder
        if(diagMap.size === 0){
            if(placeholder) placeholder.style.display = 'block';
        } else {
            if(placeholder) placeholder.style.display = 'none';
        }

        // Attach click handler once to toggle details visibility using event delegation
        if(!container.dataset.listenerAttached){
            // Use 'pointerdown' so the event fires immediately on press, avoiding refresh race conditions
            container.addEventListener('pointerdown', (e)=>{
                const arrow = e.target.closest('.toggle-arrow');
                if(arrow){
                    const summaryRow = arrow.closest('tr.diag-summary');
                    if(summaryRow){
                        const detailsRow = summaryRow.nextElementSibling;
                        if(detailsRow && detailsRow.classList.contains('diag-details')){
                            const currentlyOpen = detailsRow.style.display !== 'none';
                            if(currentlyOpen){
                                detailsRow.style.display = 'none';
                                openDetails.delete(summaryRow.dataset.name);
                                arrow.textContent = '▼';
                            }else{
                                detailsRow.style.display = 'table-row';
                                openDetails.add(summaryRow.dataset.name);
                                arrow.textContent = '▲';
                            }
                        }
                    }
                }
            });
            container.dataset.listenerAttached = '1';
        }
    }

    function handleDiagnostics(msg){
        console.log('[Diagnostics] Received DiagnosticArray message', msg);
        if(!msg.status){
            console.warn('[Diagnostics] Message has no status field');
            return;
        }
        // Store or overwrite entry keyed only by the diagnostic name.
        // This merges statuses that share the same name but differ in hardware_id,
        // ensuring the table shows just one row per unique name.
        msg.status.forEach(stat=>{
            const enriched = { ...stat, receivedAt: Date.now() };
            diagMap.set(stat.name, enriched);
        });
        console.log(`[Diagnostics] Stored ${msg.status.length} status entries (total unique=${diagMap.size})`);
        renderTable();
    }

    function subscribe(){
        if(subscribed){
            console.log('[Diagnostics] Already subscribed, skipping');
            return;
        }
        if(typeof rosConnection === 'undefined' || !rosConnection){
            console.warn('[Diagnostics] rosConnection not yet created');
            return;
        }
        if(rosConnection.readyState!==WebSocket.OPEN){
            console.warn('[Diagnostics] rosConnection not open (state='+rosConnection.readyState+')');
            return;
        }
        subscriptionId = 'diag_'+Date.now();
        const subMsg = { op:'subscribe', topic:'/diagnostics', type:'diagnostic_msgs/msg/DiagnosticArray', id: subscriptionId };
        console.log('[Diagnostics] Sending subscribe message to /diagnostics', subMsg);
        rosConnection.send(JSON.stringify(subMsg));

        diagHandler = (e)=>{
            if(!e.detail){return;}
            if(e.detail.topic==='/diagnostics'){
                console.log('[Diagnostics] topic-message event for /diagnostics received');
                handleDiagnostics(e.detail.msg);
            }
        };
        document.addEventListener('topic-message', diagHandler);
        subscribed = true;
        console.log('[Diagnostics] Subscription complete');
    }

    function unsubscribe(){
        if(!subscribed) return;
        try{
            if(typeof rosConnection!=='undefined' && rosConnection && rosConnection.readyState===WebSocket.OPEN){
                const unsubMsg = {op:'unsubscribe', topic:'/diagnostics', id: subscriptionId};
                console.log('[Diagnostics] Sending unsubscribe', unsubMsg);
                rosConnection.send(JSON.stringify(unsubMsg));
            }
        }catch(err){console.error('[Diagnostics] Error during unsubscribe', err);}
        if(diagHandler){
            document.removeEventListener('topic-message', diagHandler);
            diagHandler = null;
        }
        subscribed=false;
        subscriptionId=null;
        console.log('[Diagnostics] Unsubscribed from /diagnostics');
    }

    // show when section activated
    document.addEventListener('section-diagnostics-activated', ()=>{
        subscribe();
        renderTable();
    });

    document.addEventListener('section-deactivated', (e)=>{
        if(e.detail && e.detail.sectionId==='diagnostics-section'){
            unsubscribe();
        }
    });

    // Attempt to subscribe after connection established if diagnostics section currently active
    document.addEventListener('ros-connected', ()=>{
        const diagSection = document.getElementById('diagnostics-section');
        if(diagSection && diagSection.classList.contains('is-active')){
            subscribe();
        }
    });
})();
