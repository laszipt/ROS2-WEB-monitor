/**
 * ros-connection.js
 * Handles WebSocket connection to ROS2 bridge
 */

// Configuration
// Build the WebSocket URL dynamically so it works from any device (LAN/Wi-Fi)
// Use the current page's hostname; if served via IPv6/port mapping it still works.
const ROSBRIDGE_URL = `ws://${location.hostname}:9090`;
const MAX_RECONNECT_ATTEMPTS = 5;
const RECONNECT_INTERVAL = 2000; // 2 seconds between reconnect attempts
const INITIAL_CONNECTION_DELAY = 1500; // Wait longer before the first connection attempt

// Global WebSocket connection
let rosConnection = null;
let reconnectAttempts = 0;
let reconnectTimer = null;
let isReconnecting = false;
let roslibLoaded = false;
let allResourcesLoaded = false;
let connectionAttemptMade = false; // Track if we've tried connecting
let allowConnectionAttempts = false; // Flag to control connection attempts

// Connection status
const ConnectionStatus = {
    DISCONNECTED: 'disconnected',
    CONNECTING: 'attempting',
    CONNECTED: 'connected',
    ERROR: 'disconnected'
};

/**
 * Check if ROSLIB is loaded and ready to use
 * @returns {boolean} - Whether ROSLIB is loaded
 */
function isRoslibReady() {
    return typeof ROSLIB !== 'undefined';
}

/**
 * Check if all required resources are loaded
 * @returns {Promise} - Resolves when all resources are loaded
 */
function checkAllResourcesLoaded() {
    return new Promise((resolve) => {
        // Simple check for ROSLIB readiness
        if (isRoslibReady()) {
            allResourcesLoaded = true;
            logToConsole('ROSLIB library detected, resources considered loaded');
            resolve(true);
            return;
        }
        
        logToConsole('Waiting for ROSLIB library to load...');
        
        // Check periodically for ROSLIB
        const checkInterval = setInterval(() => {
            if (isRoslibReady()) {
                clearInterval(checkInterval);
                allResourcesLoaded = true;
                logToConsole('ROSLIB library loaded successfully');
                resolve(true);
            }
        }, 100);
        
        // Set a timeout in case something goes wrong
        setTimeout(() => {
            clearInterval(checkInterval);
            if (!allResourcesLoaded) {
                logToConsole('Warning: Timed out waiting for ROSLIB to load', 'warn');
                allResourcesLoaded = true; // Force it true and try anyway
                resolve(true);
            }
        }, 8000); // 8 second timeout
    });
}

/**
 * Initialize ROS connection events and UI elements
 */
function initRosConnection() {
    // Get UI elements
    const statusIndicator = document.getElementById('status-indicator');
    const connectionText = document.getElementById('connection-text');
    const reconnectBtn = document.getElementById('reconnectBtn');

    // Set up reconnect button
    if (reconnectBtn) {
        reconnectBtn.addEventListener('click', manualReconnect);
    }

    // We no longer connect automatically here - controlled by sequential loader instead
    logToConsole('ROS connection initialization complete - waiting for sequential loader to trigger connection');
    
    // Listen for visibility change events to reconnect if needed when tab becomes visible again
    document.addEventListener('visibilitychange', function() {
        if (document.visibilityState === 'visible' && 
            (!rosConnection || rosConnection.readyState !== WebSocket.OPEN)) {
            logToConsole('Page became visible, checking connection status...');
            
            // Make sure all resources are loaded before trying to connect
            if (allResourcesLoaded) {
                if (allowConnectionAttempts) {
                    connectToROS();
                }
            } else {
                checkAllResourcesLoaded().then(() => {
                    if (allowConnectionAttempts) {
                        connectToROS();
                    }
                });
            }
        }
    });
    
    // Handle page reload validation
    window.addEventListener('pageshow', function(event) {
        // Check if the page is being restored from the bfcache (back-forward cache)
        if (event.persisted) {
            logToConsole('Page restored from cache, validating resources and connection...');
            // Re-validate resources and reconnect if needed
            checkAllResourcesLoaded().then(() => {
                if (allowConnectionAttempts && (!rosConnection || rosConnection.readyState !== WebSocket.OPEN)) {
                    connectToROS();
                }
            });
        }
    });
}

/**
 * Update connection status in UI
 * @param {string} status - Connection status (from ConnectionStatus enum)
 * @param {string} message - Optional status message
 */
function updateConnectionStatus(status, message = null) {
    const statusIndicator = document.getElementById('status-indicator');
    const connectionText = document.getElementById('connection-text');
    
    if (statusIndicator) {
        // Remove all status classes
        statusIndicator.classList.remove('connected', 'disconnected', 'attempting');
        // Add new status class
        statusIndicator.classList.add(status);
    }
    
    if (connectionText) {
        switch (status) {
            case ConnectionStatus.CONNECTED:
                connectionText.textContent = message || 'Connected';
                break;
            case ConnectionStatus.CONNECTING:
                connectionText.textContent = message || 'Connecting...';
                break;
            case ConnectionStatus.DISCONNECTED:
            case ConnectionStatus.ERROR:
                connectionText.textContent = message || 'Disconnected';
                break;
        }
    }
    
    // Trigger events that other components can listen to
    triggerEvent(`ros-${status}`, { message: message });
}

/**
 * Connect to ROS Bridge WebSocket server
 */
function connectToROS() {
    // Don't connect if not allowed
    if (!allowConnectionAttempts) {
        logToConsole("Connection attempt denied - waiting for all scripts to load", 'warn');
        return;
    }
    
    // Track that we've made an attempt
    connectionAttemptMade = true;
    
    // Check if we already have an active connection
    if (rosConnection && rosConnection.readyState === WebSocket.OPEN) {
        logToConsole('Already connected to ROS Bridge');
        return;
    }
    
    // Close any existing connection
    if (rosConnection) {
        try {
            rosConnection.close();
        } catch (error) {
            console.error("Error closing existing connection:", error);
        }
        rosConnection = null;
    }
    
    // Update UI status
    updateConnectionStatus(ConnectionStatus.CONNECTING);
    logToConsole(`Connecting to ROS Bridge at ${ROSBRIDGE_URL}...`);
    
    // Create new WebSocket connection
    try {
        rosConnection = new WebSocket(ROSBRIDGE_URL);
        
        // Set up WebSocket event handlers
        rosConnection.onopen = handleConnectionOpen;
        rosConnection.onclose = handleConnectionClose;
        rosConnection.onerror = handleConnectionError;
        rosConnection.onmessage = handleIncomingMessage;
        
    } catch (error) {
        logToConsole(`Failed to create WebSocket connection: ${error.message}`, 'error');
        updateConnectionStatus(ConnectionStatus.ERROR);
        scheduleReconnect();
    }
}

/**
 * Debug function to check all available services
 */
function checkAvailableServices() {
    if (!rosConnection || rosConnection.readyState !== WebSocket.OPEN) {
        logToConsole('Cannot check services - not connected', 'error');
        return;
    }
    
    const requestId = 'get_services_' + Date.now();
    const message = {
        op: 'call_service',
        id: requestId,
        service: '/rosapi/services',
        args: {}
    };
    
    // Log the request
    logToConsole('Checking available services...');
    
    // Send the message
    try {
        // We need to update our handleServiceResponse function to handle this message type
        // Add a one-time event listener specifically for the diagnostics
        document.addEventListener(requestId, function onceOnly(event) {
            document.removeEventListener(requestId, onceOnly);
            const response = event.detail;
            
            logToConsole('Service diagnostic response received:');
            console.log('Full response:', response);
            
            if (response && response.values) {
                if (response.values.services && Array.isArray(response.values.services)) {
                    logToConsole(`Found ${response.values.services.length} services`);
                    response.values.services.forEach(service => {
                        logToConsole(`- ${service}`);
                    });
                } else if (response.values.names && Array.isArray(response.values.names)) {
                    // Some implementations use 'names' instead of 'services'
                    logToConsole(`Found ${response.values.names.length} services`);
                    response.values.names.forEach(service => {
                        logToConsole(`- ${service}`);
                    });
                } else {
                    logToConsole('Services list not found in expected format', 'warn');
                    console.log('Response values:', response.values);
                }
            } else {
                logToConsole('Service diagnostic failed - invalid response format', 'error');
            }
        });
        
        // Now also try a direct topic list request for diagnostics
        checkAllTopicsDirectly();
        
        rosConnection.send(JSON.stringify(message));
    } catch (error) {
        logToConsole(`Error checking services: ${error.message}`, 'error');
    }
}

/**
 * Check for topics directly without going through the service layer
 */
function checkAllTopicsDirectly() {
    if (!rosConnection || rosConnection.readyState !== WebSocket.OPEN) {
        return;
    }
    
    logToConsole('Trying direct topic list...');
    
    // Skip the direct get_topics operation as it's not supported by this rosbridge implementation
    logToConsole('Skipping direct get_topics operation - not supported by this rosbridge implementation');
    
    // Instead, try to subscribe to a known topic to verify connection
    try {
        const subscribeMessage = {
            op: 'subscribe',
            id: 'connection_test_' + Date.now(),
            topic: '/parameter_events',
            type: 'rcl_interfaces/msg/ParameterEvent'
        };
        rosConnection.send(JSON.stringify(subscribeMessage));
        logToConsole('Sent test subscription to verify connection');
    } catch (error) {
        console.error('Error sending test subscription:', error);
    }
}

/**
 * Subscribe to essential ROS2 topics for basic connectivity
 */
function tryAlternativeDiscovery() {
    if (!rosConnection || rosConnection.readyState !== WebSocket.OPEN) {
        logToConsole('Cannot perform basic subscriptions - not connected', 'error');
        return;
    }
    
    logToConsole('Setting up basic ROS2 subscriptions...');
    
    // Subscribe to a topic that should always exist in ROS2 to verify connectivity
    const subscribeMessage = {
        op: 'subscribe',
        id: 'subscribe_test_' + Date.now(),
        topic: '/parameter_events',
        type: 'rcl_interfaces/msg/ParameterEvent'
    };
    
    try {
        rosConnection.send(JSON.stringify(subscribeMessage));
        logToConsole('Subscribed to /parameter_events for basic connectivity');
    } catch (error) {
        console.error('Error setting up basic subscriptions:', error);
    }
}

/**
 * Handle incoming WebSocket messages
 * @param {MessageEvent} event - Message event
 */
function handleIncomingMessage(event) {
    try {
        const message = JSON.parse(event.data);
        
        // Log all incoming messages for diagnostic purposes
        console.log('Raw WS message:', message);
        
        // Handle different types of messages
        if (message.op === 'service_response') {
            handleServiceResponse(message);
        } else if (message.op === 'publish') {
            handleTopicMessage(message);
        } else {
            // Special diagnostic logging for unexpected message types
            logToConsole(`Received unexpected message type: ${message.op}`);
            console.log('Unexpected message content:', message);
        }
    } catch (error) {
        logToConsole(`Error parsing message: ${error.message}`, 'error');
        console.error("Error parsing message:", error);
    }
}

/**
 * Handle successful WebSocket connection
 */
function handleConnectionOpen() {
    updateConnectionStatus(ConnectionStatus.CONNECTED);
    logToConsole('Connected to ROS Bridge Server');
    reconnectAttempts = 0;

    // Optional diagnostic: we keep only the lightweight connectivity check
    setTimeout(() => {
        tryAlternativeDiscovery();
    }, 2000);

    // Trigger event for other modules to respond to connection
    triggerEvent('ros-connected');
    
}

/**
 * Handle WebSocket connection close
 * @param {Event} event - Close event
 */
function handleConnectionClose(event) {
    updateConnectionStatus(ConnectionStatus.DISCONNECTED);
    logToConsole(`Connection closed. Code: ${event.code}, Reason: ${event.reason || 'No reason provided'}`);
    
    // Attempt to reconnect unless this was a clean close or manual disconnect
    const shouldReconnect = (event.code !== 1000) && (reconnectAttempts < MAX_RECONNECT_ATTEMPTS);
    
    if (shouldReconnect) {
        scheduleReconnect();
    } else if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
        logToConsole(`Maximum reconnect attempts (${MAX_RECONNECT_ATTEMPTS}) reached. Please try refreshing the page.`, 'warn');
    }
    
    rosConnection = null;
    
    // Trigger event for other modules
    triggerEvent('ros-disconnected');
}

/**
 * Schedule a reconnection attempt
 */
function scheduleReconnect() {
    reconnectAttempts++;
    isReconnecting = true;
    
    // Use exponential backoff for reconnect delay
    const delay = RECONNECT_INTERVAL * Math.min(Math.pow(1.5, reconnectAttempts - 1), 5);
    
    logToConsole(`Scheduling reconnect attempt ${reconnectAttempts}/${MAX_RECONNECT_ATTEMPTS} in ${delay/1000} seconds...`);
    
    reconnectTimer = setTimeout(connectToROS, delay);
}

/**
 * Handle WebSocket connection error
 * @param {Event} event - Error event
 */
function handleConnectionError(event) {
    updateConnectionStatus(ConnectionStatus.ERROR, 'Connection Failed');
    logToConsole("WebSocket connection error");
    console.error(event);
    
    // Schedule reconnect if we haven't exceeded max attempts
    if (reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
        scheduleReconnect();
    } else {
        logToConsole(`Maximum reconnect attempts (${MAX_RECONNECT_ATTEMPTS}) reached. Please try refreshing the page.`, 'warn');
    }
    
    // Trigger event for other modules
    triggerEvent('ros-error');
}

/**
 * Handle ROS service responses
 * @param {Object} message - Parsed WebSocket message
 */
function handleServiceResponse(message) {
    // This is a simplified, robust version to restore basic functionality.
    // It only handles the generic list events.
    // Specific event handling for features like frequency measurement will be re-added later.

    if (message.id && message.id.startsWith('get_topics')) {
        triggerEvent('topics-list-received', message);
    } else if (message.id && message.id.startsWith('get_nodes')) {
        triggerEvent('nodes-list-received', message);
    } else if (message.id && message.id.startsWith('get_services')) {
        triggerEvent('services-list-received', message);
    } else if (message.id && message.id.startsWith('get_node_details_')) {
        triggerEvent('node-details-received', message);
    } else if (message.id && message.id.startsWith('get_param_names_')) {
        triggerEvent('param-names-received', message);
    } else if (message.id && message.id.startsWith('get_param_')) {
        triggerEvent('param-value-received', message);
    } else if (message.id) {
        // Fallback for any other specific handlers that listen by the full ID
        // (e.g., get_topic_type in topics-service.js)
        triggerEvent(message.id, message);
    } else {
        logToConsole('Received service response without an ID.', 'warn');
    }
}

/**
 * Handle ROS topic message
 * @param {Object} message - Parsed WebSocket message
 */
function handleTopicMessage(message) {
    if (!message.topic) {
        logToConsole("Received topic message without topic name", 'warn');
        return;
    }
    
    // Trigger event with topic name and message
    triggerEvent(`topic-message-${message.topic}`, message);
    // Also trigger a general topic message event
    triggerEvent('topic-message', message);
}

/**
 * Send a message to the ROS Bridge WebSocket server
 * @param {Object} message - Message to send
 * @returns {boolean} - Whether the message was sent successfully
 */
function sendRosMessage(message) {
    if (!rosConnection || rosConnection.readyState !== WebSocket.OPEN) {
        logToConsole("Cannot send message: WebSocket not connected", 'error');
        return false;
    }
    
    try {
        rosConnection.send(JSON.stringify(message));
        return true;
    } catch (error) {
        logToConsole(`Error sending message: ${error.message}`, 'error');
        console.error("Error sending message:", error);
        return false;
    }
}

/**
 * Call a ROS service
 * @param {string} serviceName - ROS service name
 * @param {Object} args - Service arguments
 * @param {string} requestId - Unique request ID
 * @returns {boolean} - Whether the request was sent successfully
 */
function callRosService(serviceName, args = {}, requestId = null) {
    // Generate request ID if not provided
    if (!requestId) {
        requestId = `service_call_${serviceName.replace(/\//g, '_')}_${Date.now()}`;
    }
    
    const request = {
        op: 'call_service',
        service: serviceName,
        args: args,
        id: requestId
    };
    
    logToConsole(`Calling service: ${serviceName}`, 'info');
    return sendRosMessage(request);
}

/**
 * Trigger a custom event for inter-module communication
 * @param {string} eventName - Event name
 * @param {Object} data - Event data
 */
function triggerEvent(eventName, data = null) {
    const event = new CustomEvent(eventName, { detail: data });
    document.dispatchEvent(event);
    // Log debug events
    //logToConsole(`Event triggered: ${eventName}`, 'debug');
}

/**
 * Log message to console and UI console element
 * @param {string} message - Message to log
 * @param {string} level - Log level (info, warn, error, debug)
 */
function logToConsole(message, level = 'info') {
    const consoleOutput = document.getElementById('console-output');
    if (!consoleOutput) return;
    
    const timestamp = new Date().toLocaleTimeString();
    const formattedMessage = `[${timestamp}] ${message}`;
    
    // Log to browser console based on level
    switch (level) {
        case 'error':
            console.error(formattedMessage);
            break;
        case 'warn':
            console.warn(formattedMessage);
            break;
        case 'debug':
            console.debug(formattedMessage);
            break;
        case 'info':
        default:
            console.info(formattedMessage);
    }
    
    // Add to UI console
    const logEntry = document.createElement('div');
    logEntry.textContent = formattedMessage;
    logEntry.classList.add(`log-${level}`);
    
    consoleOutput.appendChild(logEntry);
    consoleOutput.scrollTop = consoleOutput.scrollHeight;
    
    // Limit console length
    while (consoleOutput.childNodes.length > 100) {
        consoleOutput.removeChild(consoleOutput.firstChild);
    }
}

// Set up connection initialization
document.addEventListener('DOMContentLoaded', () => {
    // Initialize UI elements only, but don't connect yet
    initRosConnection();
    
    logToConsole('ROS connection module initialized, waiting for sequential loader to complete');
});

// Add manual reconnect trigger for the button
function manualReconnect() {
    reconnectAttempts = 0;
    clearTimeout(reconnectTimer);
    connectionAttemptMade = false;
    connectToROS();
}

// Add global connection function that can be called from other scripts
window.startRosConnection = function() {
    if (window.ALL_SCRIPTS_LOADED !== true) {
        logToConsole('Connection attempt blocked - not all scripts are loaded yet', 'warn');
        return;
    }
    
    allowConnectionAttempts = true;
    logToConsole('Manual connection trigger received - all scripts loaded, connecting now');
    
    if (!connectionAttemptMade) {
        connectToROS();
    }
};
