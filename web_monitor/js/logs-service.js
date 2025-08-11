// logs-service.js - Ultra minimal version with debugging

// Listen for logs section activation
let listenerAdded = false;
// Add initialized flag to track initialization state
let initialized = false;

function onLogsSectionActivated(event) {
    console.log('[LOGS] Logs section activated. Fetching logs.');
    fetchLogNodes();
}

// Cache for node logs
const nodeLogsCache = {};

/**
 * Fetch log nodes data from the API
 */
function fetchLogNodes() {
    const timestamp = Date.now();
    fetch(`/api/log-nodes?t=${timestamp}`)
        .then(response => response.json())
        .then(data => {
            // The response is a direct array of nodes.
            // Store the log file paths in the cache
            if (Array.isArray(data)) {
                data.forEach(node => {
                    const key = `${node.node_name}_${node.pid}`;
                    nodeLogsCache[key] = {
                        nodeName: node.node_name,
                        pid: node.pid
                    };
                });
            }
            
            // Format the data as a table
            const logsContent = document.getElementById('logs-content');
            if (logsContent && Array.isArray(data) && data.length > 0) {
                // Create table
                let tableHtml = `
                    <table class="table is-fullwidth is-striped is-hoverable">
                        <thead>
                            <tr>
                                <th>Started</th>
                                <th>Node Name</th>
                                <th>PID</th>
                                <th>Status</th>
                                <th>Actions</th>
                            </tr>
                        </thead>
                        <tbody>
                `;
                
                // Add rows for each node
                data.forEach(node => {
                    // Format timestamp for better readability (Unix timestamp is in seconds)
                    const timestamp = new Date(parseFloat(node.timestamp) * 1000).toLocaleString();
                    
                    // Determine status display
                    const statusHtml = node.is_alive ? 
                        '<span class="has-text-success"><i class="fas fa-check"></i> Running</span>' : 
                        '<span class="has-text-danger"><i class="fas fa-times"></i> Terminated</span>';
                    
                    // Add Kill button only for running nodes
                    const actionsHtml = node.is_alive ? 
                        `<button class="button is-small is-danger" onclick="killNode('${node.node_name}', ${node.pid})">
                            <span class="icon is-small">
                                <i class="fas fa-times"></i>
                            </span>
                            <span>Kill</span>
                        </button>` : 
                        '';
                    
                    tableHtml += `
                        <tr>
                            <td>${timestamp}</td>
                            <td><a href="#" onclick="viewNodeLog('${node.node_name}', ${node.pid}); return false;">${node.node_name}</a></td>
                            <td>${node.pid}</td>
                            <td>${statusHtml}</td>
                            <td>${actionsHtml}</td>
                        </tr>
                    `;
                });
                
                tableHtml += `
                        </tbody>
                    </table>
                `;
                
                logsContent.innerHTML = tableHtml;
                
            } else if (logsContent) {
                // Handle case where data is empty or not an array
                logsContent.innerHTML = '<div class="notification is-info">No running nodes found or data is in an unexpected format.</div>';
            }
        })
        .catch(error => {
            console.error('Error fetching log nodes:', error);
            const logsContent = document.getElementById('logs-content');
            if (logsContent) {
                logsContent.innerHTML = `<div class="notification is-danger">Error loading logs: ${error.message}</div>`;
            }
        });
}

/**
 * View the log file for a specific node
 * @param {string} nodeName - Name of the node
 * @param {number} pid - Process ID of the node
 */
function viewNodeLog(nodeName, pid) {
    const key = `${nodeName}_${pid}`;
    if (!nodeLogsCache[key]) {
        showNotification('Node not found in cache', 'warning');
        return;
    }

    // Show loading modal
    showLoadingModal(`Loading log for ${nodeName} (PID: ${pid})...`);

    // Call new API endpoint that returns content for the PID
    fetch(`/api/log-nodes?pid=${pid}&t=${Date.now()}`)
        .then(resp => resp.json())
        .then(data => {
            hideLoadingModal();
            if (data && data.content) {
                showLogModal(nodeName, pid, data.content);
            } else if (data && data.error) {
                showNotification(`Error: ${data.error}`, 'danger');
            } else {
                showNotification('Unexpected response format', 'warning');
            }
        })
        .catch(err => {
            hideLoadingModal();
            showNotification(`Error fetching log content: ${err.message}`, 'danger');
        });
}

/**
 * Kill a node
 * @param {string} nodeName - Name of the node
 * @param {number} pid - Process ID of the node
 */
function killNode(nodeName, pid) {
    // Call API endpoint to kill the node
    fetch(`/api/kill-node?nodeName=${nodeName}&pid=${pid}`, { method: 'POST' })
        .then(resp => resp.json())
        .then(data => {
            if (data && data.success) {
                showNotification(`Node ${nodeName} (PID: ${pid}) killed successfully`, 'success');
                // Show a brief notification before refreshing
                setTimeout(() => {
                    // Refresh only the logs table
                    fetchLogNodes();
                }, 1000); // Wait 1 second to show the notification before refresh
            } else if (data && data.error) {
                showNotification(`Error killing node: ${data.error}`, 'danger');
            } else {
                showNotification('Unexpected response format', 'warning');
            }
        })
        .catch(err => {
            showNotification(`Error killing node: ${err.message}`, 'danger');
        });
}

/**
 * Show a loading modal
 * @param {string} message - Loading message to display
 */
function showLoadingModal(message) {
    // Create modal if it doesn't exist
    let modal = document.getElementById('loading-modal');
    if (!modal) {
        modal = document.createElement('div');
        modal.id = 'loading-modal';
        modal.className = 'modal is-active';
        modal.innerHTML = `
            <div class="modal-background"></div>
            <div class="modal-content">
                <div class="box has-text-centered">
                    <div class="loader is-loading"></div>
                    <p id="loading-message" class="mt-3">${message}</p>
                </div>
            </div>
        `;
        document.body.appendChild(modal);
    } else {
        // Update existing modal
        modal.classList.add('is-active');
        document.getElementById('loading-message').textContent = message;
    }
}

/**
 * Hide the loading modal
 */
function hideLoadingModal() {
    const modal = document.getElementById('loading-modal');
    if (modal) {
        modal.classList.remove('is-active');
    }
}

/**
 * Show a notification message
 * @param {string} message - Message to display
 * @param {string} type - Type of notification (info, success, warning, danger)
 */
function showNotification(message, type = 'info') {
    // Create notification element
    const notification = document.createElement('div');
    notification.className = `notification is-${type}`;
    notification.innerHTML = `
        <button class="delete"></button>
        ${message}
    `;
    
    // Add to notifications container or create one
    let container = document.getElementById('notifications-container');
    if (!container) {
        container = document.createElement('div');
        container.id = 'notifications-container';
        container.style.position = 'fixed';
        container.style.top = '1rem';
        container.style.right = '1rem';
        container.style.zIndex = '9999';
        document.body.appendChild(container);
    }
    
    container.appendChild(notification);
    
    // Add close button functionality
    notification.querySelector('.delete').addEventListener('click', function() {
        notification.remove();
    });
    
    // Auto-remove after 5 seconds
    setTimeout(() => {
        notification.remove();
    }, 5000);
}

/**
 * Show a modal with log content
 * @param {string} nodeName - Name of the node
 * @param {number} pid - Process ID of the node
 * @param {string} content - Log file content
 */
function showLogModal(nodeName, pid, content) {
    // Create modal if it doesn't exist
    let modal = document.getElementById('log-content-modal');
    if (!modal) {
        modal = document.createElement('div');
        modal.id = 'log-content-modal';
        modal.className = 'modal';
        modal.innerHTML = `
            <div class="modal-background"></div>
            <div class="modal-card" style="width: 90vw; max-width: 1200px;">
                <header class="modal-card-head">
                    <p class="modal-card-title" id="log-modal-title"></p>
                    <button class="delete" aria-label="close"></button>
                </header>
                <section class="modal-card-body">
                    <div class="content">
                        <pre id="log-content" style="max-height: 70vh; overflow: auto; white-space: pre-wrap;"></pre>
                    </div>
                </section>
                <footer class="modal-card-foot">
                    <button class="button is-info" id="download-log-btn">Download Log</button>
                    <button class="button" id="close-log-modal-btn">Close</button>
                </footer>
            </div>
        `;
        document.body.appendChild(modal);
        
        // Add event listeners
        modal.querySelector('.delete').addEventListener('click', () => {
            modal.classList.remove('is-active');
        });
        
        modal.querySelector('#close-log-modal-btn').addEventListener('click', () => {
            modal.classList.remove('is-active');
        });
        
        modal.querySelector('#download-log-btn').addEventListener('click', () => {
            downloadLogContent(nodeName, content);
        });
    }
    
    // Update modal content
    document.getElementById('log-modal-title').textContent = `Log for ${nodeName} (PID: ${pid})`;
    document.getElementById('log-content').textContent = content;
    
    // Show the modal
    modal.classList.add('is-active');
}

/**
 * Download log content as a file
 * @param {string} nodeName - Name of the node
 * @param {string} content - Log file content
 */
function downloadLogContent(nodeName, content) {
    const filename = `${nodeName.replace(/\//g, '_')}_log_${Date.now()}.txt`;
    const blob = new Blob([content], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    a.click();
    
    URL.revokeObjectURL(url);
}

function initLogsService() {
    // Check if already initialized to prevent duplicate initialization
    if (initialized) {
        console.log('[LOGS] logs-service already initialized, skipping');
        return;
    }
    
    if (!listenerAdded) {
        // Listen for when the section is shown
        document.addEventListener('section-logs-activated', onLogsSectionActivated);
        listenerAdded = true;
    }
    
    window.debugFetchLogNodes = fetchLogNodes;
    console.log('[LOGS] logs-service initialized');
    
    // Add CSS for notifications container
    const style = document.createElement('style');
    style.textContent = `
        #notifications-container {
            display: flex;
            flex-direction: column;
            gap: 0.5rem;
            max-width: 25rem;
        }
        
        .notification {
            margin-bottom: 0 !important;
            box-shadow: 0 2px 4px rgba(0,0,0,0.2);
        }
        
        .loader {
            border: 4px solid #f3f3f3;
            border-top: 4px solid #3498db;
            border-radius: 50%;
            width: 50px;
            height: 50px;
            animation: spin 1s linear infinite;
            margin: 0 auto;
        }
        
        @keyframes spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
        }
    `;
    document.head.appendChild(style);
    
    // Mark as initialized
    initialized = true;

    // Immediately populate the table once on load so it is ready when the user opens the Logs section
    fetchLogNodes();
}

// The service is now initialized manually from newindex.html
