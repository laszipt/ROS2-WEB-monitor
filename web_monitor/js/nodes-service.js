/**
 * nodes-service.js
 * Handles interaction with ROS nodes service
 */

// Initialize event listeners when document is ready
document.addEventListener('DOMContentLoaded', function() {
    initNodesService();
});

// Initialize nodes service
function initNodesService() {
    // Set up event listeners for nodes service
    document.addEventListener('ros-connected', function() {
        // Don't automatically query nodes at startup
        // Only query if nodes section is currently active
        const nodesSection = document.getElementById('nodes-section');
        if (nodesSection && nodesSection.classList.contains('is-active')) {
            console.log('[Nodes Service] Nodes section is active, querying nodes list');
            refreshNodesList();
        } else {
            console.log('[Nodes Service] Nodes section not active, skipping initial nodes query');
        }
    });
    
    document.addEventListener('nodes-list-received', function(event) {
        handleNodesResponse(event.detail);
    });
    
    document.addEventListener('node-details-received', function(event) {
        handleNodeDetailsResponse(event.detail);
    });
    
    document.addEventListener('param-names-received', function(event) {
        handleParamNamesResponse(event.detail);
    });
    
    document.addEventListener('param-value-received', function(event) {
        handleParamValueResponse(event.detail);
    });
    
    document.addEventListener('section-nodes-activated', function() {
        console.log('[Nodes Service] Nodes section activated, refreshing nodes list');
        if (rosConnection && rosConnection.readyState === WebSocket.OPEN) {
            refreshNodesList();
        }
    });
    
    logToConsole('Nodes service initialized');
}

/**
 * Refresh the list of ROS nodes
 */
function refreshNodesList() {
    if (!rosConnection || rosConnection.readyState !== WebSocket.OPEN) {
        showError('nodes-list', 'WebSocket connection is not open');
        return;
    }
    
    // Show loading state
    showLoading('nodes-list', 'Loading ROS nodes...');
    
    // Call the ROS service to get nodes
    const success = callRosService('/rosapi/nodes', {}, 'get_nodes_' + Date.now());
    
    if (!success) {
        hideLoading('nodes-list');
        showError('nodes-list', 'Failed to send node list request');
    }
    
    // Set timeout for response
    setTimeout(function() {
        const nodesList = document.getElementById('nodes-list');
        if (nodesList && nodesList.querySelector('.loader')) {
            // If still showing loading after timeout, show error
            showError('nodes-list', 'Request timed out. No response received.');
        }
    }, 5000);
}

/**
 * Handle response from the nodes service
 * @param {Object} response - Service response message
 */
function handleNodesResponse(response) {
    // Add detailed logging for debugging
    console.log("Nodes response received:", response);
    
    // More robust response validation
    if (!response) {
        showError('nodes-list', 'No response received');
        return;
    }
    
    if (!response.values) {
        showError('nodes-list', 'Missing values in response');
        console.error("Invalid nodes response format (missing values):", response);
        return;
    }
    
    // In ROS2, the field might be 'nodes', but in some implementations it could be 'names'
    const nodes = response.values.nodes || response.values.names || [];
    
    if (!Array.isArray(nodes)) {
        showError('nodes-list', 'Invalid response format (nodes is not an array)');
        console.error("Invalid nodes array:", nodes);
        return;
    }
    
    logToConsole(`Received ${nodes.length} nodes`);
    
    // Generate table rows for nodes
    let tableContent = '';
    if (nodes.length === 0) {
        tableContent = '<tr><td class="has-text-centered">No nodes found</td></tr>';
    } else {
        nodes.sort().forEach(node => {
            tableContent += `
                <tr>
                    <td>
                        <a href="#" class="node-name" data-node="${node}">
                            ${node}
                        </a>
                    </td>
                </tr>
            `;
        });
    }
    
    // Update the table content
    hideLoading('nodes-list', tableContent);
    
    // Add event listeners to node name links
    setupNodeActionButtons();
}

/**
 * Set up node action buttons
 */
function setupNodeActionButtons() {
    // Node name links
    const nodeLinks = document.querySelectorAll('.node-name');
    nodeLinks.forEach(link => {
        link.addEventListener('click', function(e) {
            e.preventDefault(); // Prevent default anchor behavior
            const nodeName = this.getAttribute('data-node');
            showNodeInfo(nodeName);
        });
    });
}

/**
 * Show detailed information about a ROS node
 * @param {string} nodeName - Name of the node
 */
function showNodeInfo(nodeName) {
    logToConsole(`Showing info for node: ${nodeName}`);
    
    // Create modal for node info
    const modalHtml = `
        <div class="modal is-active" id="node-info-modal">
            <div class="modal-background"></div>
            <div class="modal-card">
                <header class="modal-card-head">
                    <p class="modal-card-title">Node Information: ${nodeName}</p>
                    <button class="delete close-modal" aria-label="close"></button>
                </header>
                <section class="modal-card-body">
                    <div class="tabs">
                        <ul>
                            <li class="is-active"><a data-tab="topics">Topics</a></li>
                            <li><a data-tab="services">Services</a></li>
                            <li><a data-tab="parameters">Parameters</a></li>
                        </ul>
                    </div>
                    
                    <div class="tab-content" id="tab-topics">
                        <div class="field">
                            <label class="label">Published Topics</label>
                            <div class="content" id="node-published-topics">
                                <div class="is-loading has-text-centered">
                                    Loading published topics...
                                </div>
                            </div>
                        </div>
                        <div class="field">
                            <label class="label">Subscribed Topics</label>
                            <div class="content" id="node-subscribed-topics">
                                <div class="is-loading has-text-centered">
                                    Loading subscribed topics...
                                </div>
                            </div>
                        </div>
                    </div>
                    
                    <div class="tab-content is-hidden" id="tab-services">
                        <div class="field">
                            <label class="label">Services Provided by Node</label>
                            <div class="tags" id="node-services-tags">
                                <span class="tag is-light is-info">Loading services...</span>
                            </div>
                        </div>
                    </div>
                    
                    <div class="tab-content is-hidden" id="tab-parameters">
                        <div class="field">
                            <label class="label">Node Parameters</label>
                            <div id="node-parameters-content">
                                <div class="notification is-info is-light">
                                    Loading parameters...
                                    <progress class="progress is-small is-primary" max="100"></progress>
                                </div>
                            </div>
                        </div>
                    </div>
                </section>
                <footer class="modal-card-foot">
                    <button class="button close-modal">Close</button>
                </footer>
            </div>
        </div>
    `;
    
    // Add modal to document
    document.body.insertAdjacentHTML('beforeend', modalHtml);
    
    // Set up modal close buttons
    const closeButtons = document.querySelectorAll('.close-modal');
    closeButtons.forEach(btn => {
        btn.addEventListener('click', function() {
            document.getElementById('node-info-modal').remove();
            // Clean up any parameters-related global variables if they were set from the info window
            if (window.infoNodeParametersMode) {
                delete window.currentParamNode;
                delete window.currentParamsList;
                delete window.paramValues;
                delete window.infoNodeParametersMode;
            }
        });
    });
    
    // Set up tab switching
    const tabLinks = document.querySelectorAll('.tabs a');
    tabLinks.forEach(link => {
        link.addEventListener('click', function() {
            // Remove active class from all tabs
            document.querySelectorAll('.tabs li').forEach(li => li.classList.remove('is-active'));
            this.parentElement.classList.add('is-active');
            
            // Hide all tab content
            document.querySelectorAll('.tab-content').forEach(content => content.classList.add('is-hidden'));
            
            // Show selected tab content
            const tabId = 'tab-' + this.getAttribute('data-tab');
            document.getElementById(tabId).classList.remove('is-hidden');
            
            // Load parameters when the parameters tab is selected
            if (tabId === 'tab-parameters') {
                showNodeParametersInInfoTab(nodeName);
            }
        });
    });
    
    // Now fetch actual node information
    fetchNodeInfo(nodeName);
}

/**
 * Show parameters for a node in the info popup tab
 * @param {string} nodeName - Name of the node
 */
function showNodeParametersInInfoTab(nodeName) {
    // Show loading indicator
    document.getElementById('node-parameters-content').innerHTML = `
        <div class="notification is-info is-light">
            Loading parameters...
            <progress class="progress is-small is-primary" max="100"></progress>
        </div>
    `;
    
    // REUSE EXISTING PARAMETER FUNCTIONALITY
    // Store node name in global variables - same as in showNodeParameters
    window.currentParamNode = nodeName;
    window.currentParamsList = []; // Will be populated by handleParamNamesResponse
    window.paramValues = {}; // Will be populated by handleParamValueResponse
    
    // Create the parameters table with the SAME ID as the original parameters popup
    // This will make the existing handlers update it automatically
    document.getElementById('node-parameters-content').innerHTML = `
        <table class="table is-fullwidth is-striped is-narrow">
            <thead>
                <tr>
                    <th>Parameter</th>
                    <th>Value</th>
                    <th>Type</th>
                </tr>
            </thead>
            <tbody id="parameters-table-body">
                <tr>
                    <td colspan="3" class="has-text-centered">Loading parameter values...</td>
                </tr>
            </tbody>
        </table>
    `;
    
    // Fetch parameters using the existing function
    fetchNodeParameters(nodeName);
    
    // Add a retry button if it times out
    setTimeout(() => {
        if (document.getElementById('parameters-table-body') && 
            document.getElementById('parameters-table-body').innerHTML.includes('Loading parameter values')) {
            
            document.getElementById('node-parameters-content').innerHTML = `
                <div class="notification is-danger">
                    Request timed out. Unable to retrieve parameters.
                    <button class="button is-small is-info ml-2" id="retry-parameters-button">Retry</button>
                </div>
            `;
            
            document.getElementById('retry-parameters-button').addEventListener('click', () => {
                showNodeParametersInInfoTab(nodeName);
            });
        }
    }, 8000);
}

/**
 * Fetch detailed information about a node
 * @param {string} nodeName - Name of the node
 */
function fetchNodeInfo(nodeName) {
    // Call the service to get node details
    callRosService('/rosapi/node_details', {node: nodeName}, 'get_node_details_' + Date.now());
}

/**
 * Handle node details response
 * @param {Object} message - The response message from the service call
 */
function handleNodeDetailsResponse(message) {
    // Add detailed logging for debugging
    console.log("Node details response received:", message);
    
    // More robust validation
    if (!message) {
        document.getElementById('node-published-topics').innerHTML = '<p class="has-text-danger">No response received</p>';
        document.getElementById('node-subscribed-topics').innerHTML = '<p class="has-text-danger">No response received</p>';
        document.getElementById('node-services-tags').innerHTML = '<p class="has-text-danger">No response received</p>';
        return;
    }
    
    if (!message.values) {
        document.getElementById('node-published-topics').innerHTML = '<p class="has-text-danger">Missing values in response</p>';
        document.getElementById('node-subscribed-topics').innerHTML = '<p class="has-text-danger">Missing values in response</p>';
        document.getElementById('node-services-tags').innerHTML = '<p class="has-text-danger">Missing values in response</p>';
        console.error("Invalid node details format (missing values):", message);
        return;
    }
    
    // Handle topics with more robust fallbacks
    const subscribedTopics = message.values.subscribing || message.values.subscriptions || [];
    
    const publishedTopics = message.values.publishing || message.values.publications || [];

    // Handle published topics
    if (publishedTopics.length > 0) {
        let tagsHtml = '';
        publishedTopics.forEach(topic => {
            tagsHtml += `<span class="tag is-primary is-medium has-text-white mr-1 mb-1" style="background-color: #1a73e8;">${topic}</span>`;
        });
        document.getElementById('node-published-topics').innerHTML = `<div class="tags">${tagsHtml}</div>`;
    } else {
        document.getElementById('node-published-topics').innerHTML = '<p>No published topics</p>';
    }

    // Handle subscribed topics
    if (subscribedTopics.length > 0) {
        let tagsHtml = '';
        subscribedTopics.forEach(topic => {
            tagsHtml += `<span class="tag is-info is-medium has-text-white mr-1 mb-1" style="background-color: #0d652d;">${topic}</span>`;
        });
        document.getElementById('node-subscribed-topics').innerHTML = `<div class="tags">${tagsHtml}</div>`;
    } else {
        document.getElementById('node-subscribed-topics').innerHTML = '<p>No subscribed topics</p>';
    }
    
    // Handle services
    const services = message.values.services || [];
    
    // Handle service tags
    if (services.length > 0) {
        let tagsHtml = '';
        services.forEach(service => {
            tagsHtml += `<span class="tag is-medium has-text-white mr-1 mb-1" style="background-color: #8B4513;">${service}</span>`;
        });
        document.getElementById('node-services-tags').innerHTML = `<div class="tags">${tagsHtml}</div>`;
    } else {
        document.getElementById('node-services-tags').innerHTML = '<p>No services found</p>';
    }
}

/**
 * Fetch parameter names for a node
 * @param {string} nodeName - Name of the node
 */
function fetchNodeParameters(nodeName) {
    // For ROS2, we need to use the list_parameters service directly
    // The correct format for ROS2 is to use the node's own list_parameters service
    const serviceCallId = 'get_param_names_' + nodeName.replace(/\//g, '_') + '_' + Date.now();
    logToConsole(`Querying parameters for node ${nodeName} using service`, 'info');
    
    // First try using node's service directly
    callRosService(`${nodeName}/list_parameters`, {}, serviceCallId);
    
    // Set timeout to show error if response isn't received
    setTimeout(() => {
        if (document.getElementById('parameters-table-body') && 
            document.getElementById('parameters-table-body').innerHTML.includes('Loading parameter values')) {
            showParameterError('No response received from parameter service. The node might not expose parameters.');
        }
    }, 5000);
}

/**
 * Handle parameter names response
 * @param {Object} message - Service response message
 */
function handleParamNamesResponse(message) {
    // Log the full message to debug
    logToConsole(`Parameter names response: ${JSON.stringify(message.values)}`, 'debug');
    
    // Check for ROS2 format with result.names
    if (!message.values || (!message.values.result && !message.values.names)) {
        showParameterError('Failed to get parameter names. The node might not expose parameters.');
        return;
    }
    
    // ROS2 services return parameters in different formats depending on the implementation
    // Try to handle both formats: direct names array or nested in result object
    let paramNames = [];
    if (message.values.result && Array.isArray(message.values.result.names)) {
        paramNames = message.values.result.names;
    } else if (Array.isArray(message.values.names)) {
        paramNames = message.values.names;
    }
    
    logToConsole(`Received ${paramNames.length} parameters for node`);
    
    if (paramNames.length === 0) {
        showNoParameters();
        return;
    }
    
    // Store parameter names for later use
    window.currentParamsList = paramNames;
    window.paramValues = {};
    
    // In ROS2, we get parameter values by node name and parameter names
    const nodeName = window.currentParamNode;
    if (!nodeName) {
        showParameterError('Node name not found');
        return;
    }
    
    // Get all parameter values at once using get_parameters service
    logToConsole(`Calling service: ${nodeName}/get_parameters`);
    callRosService(`${nodeName}/get_parameters`, {
        names: paramNames
    }, 'get_param_value_batch_' + Date.now());
    
    // Start with empty table
    const paramsContent = document.getElementById('node-params-content');
    paramsContent.innerHTML = `
        <table class="table is-fullwidth is-striped is-narrow">
            <thead>
                <tr>
                    <th>Parameter</th>
                    <th>Value</th>
                    <th>Type</th>
                </tr>
            </thead>
            <tbody id="parameters-table-body">
                <tr>
                    <td colspan="3" class="has-text-centered">
                        Loading parameter values...
                        <progress class="progress is-small is-primary" max="100"></progress>
                    </td>
                </tr>
            </tbody>
        </table>
    `;
    
    // Set timeout to show error if values aren't received
    setTimeout(() => {
        if (document.getElementById('parameters-table-body') && 
            document.getElementById('parameters-table-body').innerHTML.includes('Loading parameter values')) {
            showParameterError('Timeout while waiting for parameter values');
        }
    }, 5000);
}

/**
 * Handle parameter value response
 * @param {Object} message - Service response message
 */
function handleParamValueResponse(message) {
    if (!message.id || !message.values) {
        return;
    }
    
    // Log the full response to debug
    logToConsole(`Parameter value response: ${JSON.stringify(message.values)}`, 'debug');
    
    // For batch parameter responses
    if (message.id.startsWith('get_param_value_batch_') || message.id.includes('/get_parameters')) {
        // Various possible response formats from different ROS2 bridge implementations
        let paramValues = [];
        
        // Try different possible paths to the parameter values
        if (message.values.result && message.values.result.values) {
            paramValues = message.values.result.values;
        } else if (message.values.values) {
            paramValues = message.values.values;
        } else if (message.values.result && Array.isArray(message.values.result)) {
            paramValues = message.values.result;
        } else {
            // Check for direct object with parameter values
            if (typeof message.values === 'object' && !Array.isArray(message.values)) {
                // Some implementations might return an object with parameter values directly
                // Create param values from the current parameter list
                const params = window.currentParamsList || [];
                paramValues = params.map(param => {
                    const shortName = param.split('/').pop();
                    return message.values[shortName] || null;
                });
            }
        }
        
        if (paramValues.length === 0) {
            showParameterError('Could not parse parameter values from response');
            return;
        }
        
        const params = window.currentParamsList || [];
        
        // If params and values arrays don't match, try to match by name
        if (params.length !== paramValues.length) {
            logToConsole(`Warning: Parameter names (${params.length}) and values (${paramValues.length}) count mismatch`, 'warn');
            // Create a map of parameter names to values by best effort
            window.paramValues = {};
            for (let i = 0; i < Math.min(params.length, paramValues.length); i++) {
                window.paramValues[params[i]] = paramValues[i];
            }
        } else {
            // Create a map of parameter names to values
            window.paramValues = {};
            for (let i = 0; i < params.length; i++) {
                window.paramValues[params[i]] = paramValues[i];
            }
        }
        
        // Update the table
        updateParametersTable();
        return;
    }
    
    // Individual parameter responses (fallback method)
    const match = message.id.match(/get_param_value_(.+)_\d+$/);
    if (!match || !match[1]) {
        return;
    }
    
    const encodedParamName = match[1];
    const paramName = window.currentParamsList.find(name => 
        encodedParamName === name.replace(/\//g, '_')
    );
    
    if (!paramName) {
        return;
    }
    
    // Store parameter value
    const paramValue = message.values.value;
    window.paramValues[paramName] = paramValue;
    
    // If we have all parameters, update the table
    if (Object.keys(window.paramValues).length === window.currentParamsList.length) {
        updateParametersTable();
    }
}

/**
 * Update parameters table with all collected values
 */
function updateParametersTable() {
    const params = window.currentParamsList;
    const values = window.paramValues;
    
    if (!params || !values || params.length === 0) {
        return;
    }
    
    logToConsole(`Updating parameters table with ${params.length} parameters`);
    
    let tableContent = '';
    
    // Sort parameters by name for better display
    params.sort().forEach(param => {
        const value = values[param];
        let displayValue = '';
        let typeLabel = '';
        
        // Format value based on type
        if (value === null || value === undefined) {
            displayValue = '<em>null</em>';
            typeLabel = 'null';
        } else if (typeof value === 'boolean') {
            displayValue = value ? 'true' : 'false';
            typeLabel = 'boolean';
        } else if (typeof value === 'number') {
            displayValue = value;
            typeLabel = Number.isInteger(value) ? 'integer' : 'float';
        } else if (typeof value === 'string') {
            displayValue = `"${escapeHtml(value)}"`;
            typeLabel = 'string';
        } else if (Array.isArray(value)) {
            displayValue = `[${value.length} items]`;
            typeLabel = 'array';
        } else if (typeof value === 'object') {
            // Handle ROS2 Parameter object structure with type field
            if (value.type !== undefined) {
                // ROS2 uses numeric type codes:
                // 1: boolean, 2: integer, 3: double, 4: string, etc.
                switch (value.type) {
                    case 1: // PARAMETER_BOOL
                        displayValue = value.bool_value ? 'true' : 'false';
                        typeLabel = 'boolean';
                        break;
                    case 2: // PARAMETER_INTEGER
                        displayValue = value.integer_value;
                        typeLabel = 'integer';
                        break;
                    case 3: // PARAMETER_DOUBLE
                        displayValue = value.double_value;
                        typeLabel = 'float';
                        break;
                    case 4: // PARAMETER_STRING
                        displayValue = `"${escapeHtml(value.string_value)}"`;
                        typeLabel = 'string';
                        break;
                    case 5: // PARAMETER_BYTE_ARRAY
                        displayValue = `[${value.byte_array_value ? value.byte_array_value.length : 0} bytes]`;
                        typeLabel = 'byte array';
                        break;
                    case 6: // PARAMETER_BOOL_ARRAY
                        displayValue = `[${value.bool_array_value ? value.bool_array_value.length : 0} items]`;
                        typeLabel = 'boolean array';
                        break;
                    case 7: // PARAMETER_INTEGER_ARRAY
                        displayValue = `[${value.integer_array_value ? value.integer_array_value.length : 0} items]`;
                        typeLabel = 'integer array';
                        break;
                    case 8: // PARAMETER_DOUBLE_ARRAY
                        displayValue = `[${value.double_array_value ? value.double_array_value.length : 0} items]`;
                        typeLabel = 'float array';
                        break;
                    case 9: // PARAMETER_STRING_ARRAY
                        displayValue = `[${value.string_array_value ? value.string_array_value.length : 0} items]`;
                        typeLabel = 'string array';
                        break;
                    default:
                        displayValue = JSON.stringify(value);
                        typeLabel = 'unknown';
                }
            } else if (value.bool_value !== undefined) {
                displayValue = value.bool_value ? 'true' : 'false';
                typeLabel = 'boolean';
            } else if (value.integer_value !== undefined) {
                displayValue = value.integer_value;
                typeLabel = 'integer';
            } else if (value.double_value !== undefined) {
                displayValue = value.double_value;
                typeLabel = 'float';
            } else if (value.string_value !== undefined) {
                displayValue = `"${escapeHtml(value.string_value)}"`;
                typeLabel = 'string';
            } else if (value.value !== undefined) {
                // Handle wrapped values
                const innerValue = value.value;
                if (typeof innerValue === 'boolean') {
                    displayValue = innerValue ? 'true' : 'false';
                    typeLabel = 'boolean';
                } else if (typeof innerValue === 'number') {
                    displayValue = innerValue;
                    typeLabel = Number.isInteger(innerValue) ? 'integer' : 'float';
                } else if (typeof innerValue === 'string') {
                    displayValue = `"${escapeHtml(innerValue)}"`;
                    typeLabel = 'string';
                } else if (Array.isArray(innerValue)) {
                    displayValue = `[${innerValue.length} items]`;
                    typeLabel = 'array';
                } else {
                    displayValue = JSON.stringify(innerValue);
                    typeLabel = typeof innerValue;
                }
            } else {
                // Unknown object format
                displayValue = JSON.stringify(value);
                typeLabel = 'object';
            }
        } else {
            displayValue = String(value);
            typeLabel = typeof value;
        }
        
        // Create table row
        tableContent += createParamTableRow(param, displayValue, typeLabel);
    });
    
    // Update the table body
    document.getElementById('parameters-table-body').innerHTML = tableContent;
}

/**
 * Show error message in parameters modal
 * @param {string} message - Error message to show
 */
function showParameterError(message) {
    const paramsContent = document.getElementById('node-params-content');
    paramsContent.innerHTML = `
        <div class="notification is-danger">
            ${message}
        </div>
    `;
}

/**
 * Show no parameters message in modal
 */
function showNoParameters() {
    const paramsContent = document.getElementById('node-params-content');
    paramsContent.innerHTML = `
        <div class="notification is-warning">
            No parameters found for this node.
        </div>
    `;
}

/**
 * Escape HTML special characters
 * @param {string} unsafe - String to escape
 * @returns {string} - Escaped string
 */
function escapeHtml(unsafe) {
    return unsafe
        .replace(/&/g, "&amp;")
        .replace(/</g, "&lt;")
        .replace(/>/g, "&gt;")
        .replace(/"/g, "&quot;")
        .replace(/'/g, "&#039;");
}

/**
 * Create a table row for a parameter
 * @param {string} paramName - Name of the parameter
 * @param {string} displayValue - Display value of the parameter
 * @param {string} typeLabel - Type label of the parameter
 * @returns {string} - HTML for the table row
 */
function createParamTableRow(paramName, displayValue, typeLabel) {
    // Extract short parameter name (remove node prefix)
    const shortName = paramName.split('/').pop();
    
    return `
        <tr>
            <td title="${paramName}"><code>${shortName}</code></td>
            <td><code>${displayValue}</code></td>
            <td><span class="tag is-info is-light">${typeLabel}</span></td>
        </tr>
    `;
}
