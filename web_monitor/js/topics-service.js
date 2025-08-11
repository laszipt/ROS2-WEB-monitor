/**
 * topics-service.js
 * Handles interaction with ROS topics service
 */

// Topic data cache
let topicsCache = [];
let topicTypesCache = {};
let topicSubscribersCache = {};
let topicPublishersCache = {};

// Add initialized flag to track initialization state
let topicsServiceInitialized = false;

// Topic echo subscription ID
let topicEchoSubscription = null;
let currentTopicSubscription = null;

// Frequency measurement variables
let frequencyMeasurementActive = false;
let messageTimestamps = [];
let measurementStartTime = 0;
let frequencyUpdateInterval = null;

// Measurement window in milliseconds
const MEASUREMENT_WINDOW = 5000; // 5 second window for frequency calculation

// Initialize event listeners when document is ready
document.addEventListener('DOMContentLoaded', function() {
    initTopicsService();
});

// Also initialize when topics section is activated
document.addEventListener('section-activated', function(event) {
    if (event.detail === 'topics-section') {
        if (rosConnection && rosConnection.readyState === WebSocket.OPEN) {
            refreshTopicsList();
        }
    }
});
    
// Initialize topics service
function initTopicsService() {
    // Check if already initialized to prevent duplicate initialization
    if (topicsServiceInitialized) {
        console.log('[Topics Service] Already initialized, skipping');
        return;
    }
    
    // Set up event listeners
    document.addEventListener('ros-connected', function() {
        console.log('[Topics Service] ROS connected, refreshing topic data');
        // Don't automatically query topics at startup
        // Only query if topics section is currently active
        const topicsSection = document.getElementById('topics-section');
        if (topicsSection && topicsSection.classList.contains('is-active')) {
            console.log('[Topics Service] Topics section is active, querying topics list');
            refreshTopicsList();
        } else {
            console.log('[Topics Service] Topics section not active, skipping initial topics query');
        }
    });
    
    document.addEventListener('topics-list-received', function(event) {
        handleTopicsResponse(event.detail);
    });
    

    
    document.addEventListener('subscribers-received', function(event) {
        handleSubscribersResponse(event.detail);
    });
    
    document.addEventListener('publishers-received', function(event) {
        handlePublishersResponse(event.detail);
    });
    
    logToConsole('Topics service initialized');
    
    // Mark as initialized
    topicsServiceInitialized = true;
}

/**
 * Refresh the list of ROS topics
 */
function refreshTopicsList() {
    if (!rosConnection || rosConnection.readyState !== WebSocket.OPEN) {
        showError('topics-list', 'WebSocket connection is not open');
        return;
    }

    // Reset cache
    topicsCache = [];
    topicTypesCache = {};
    topicSubscribersCache = {};
    topicPublishersCache = {};

    // Show loading state
    showLoading('topics-list', 'Loading ROS topics...');

    // Use the centralized service call function for consistency
    const requestId = 'get_topics_' + Date.now();
    const success = callRosService('/rosapi/topics', {}, requestId);

    if (success) {
        // Schedule timeout for response
        setTimeout(function() {
            const topicsList = document.getElementById('topics-list');
            if (topicsList && topicsList.querySelector('.loader')) {
                // If still showing loading after timeout, show error
                showError('topics-list', 'Request timed out. No response received.');
            }
        }, 5000);
    } else {
        hideLoading('topics-list');
        showError('topics-list', 'Failed to send topics request');
    }
}

/**
 * Handle the response from topics service
 * @param {Object} response - Response data from topics service
 */
function handleTopicsResponse(response) {
    if (!response || !response.values || !response.values.topics) {
        console.error('Invalid topics response:', response);
        showError('topics-list', 'Invalid topics response received');
        return;
    }
    
    hideLoading('topics-list');
    
    // Store topics in cache
    topicsCache = response.values.topics;
    
    // Get types for all topics
    topicsCache.forEach(topic => {
        getTopicType(topic);
    });
    
    // Display topics list
    displayTopicsList();
}

/**
 * Get type information for a specific topic. This function now handles its own
 * response instead of relying on a generic event handler.
 * @param {string} topic - Topic name
 */
function getTopicType(topic) {
    if (!rosConnection || rosConnection.readyState !== WebSocket.OPEN) {
        console.error(`[Topics Service] Cannot get topic type for ${topic}: WebSocket not connected`);
        return;
    }

    // If we already have the type or are fetching it, don't re-request.
    if (topicTypesCache[topic]) {
        return;
    }

    console.log(`[Topics Service] Requesting type for topic: ${topic}`);
    topicTypesCache[topic] = 'Fetching...'; // Mark as in-flight

    const requestId = `get_topic_type_${topic.replace(/\//g, '_')}_${Date.now()}`;

    const responseHandler = function(event) {
        document.removeEventListener(requestId, responseHandler); // Clean up listener
        const response = event.detail;

        if (response && response.values && response.values.type) {
            const type = response.values.type;
            topicTypesCache[topic] = type;
            console.log(`[Topics Service] Received type for ${topic}: ${type}`);

            // Update the main topics list UI
            const topicRows = document.querySelectorAll(`.topic-row[data-topic="${topic}"]`);
            topicRows.forEach(row => {
                const typeCell = row.querySelector('.topic-type');
                if (typeCell) typeCell.textContent = type;
            });

            // Update the topic info modal if it's open for this topic
            const topicTypeValue = document.getElementById('topic-type-value');
            const modalTitle = document.querySelector('#topic-info-modal .modal-card-title');
            if (topicTypeValue && modalTitle && modalTitle.textContent.includes(topic)) {
                topicTypeValue.textContent = type;
            }

            if (Object.keys(topicTypesCache).length === topicsCache.length) {
                displayTopicsList();
            }
        } else {
            console.error(`Invalid or missing type in response for ${topic}:`, response);
            topicTypesCache[topic] = 'Error'; // Cache failure
        }
    };

    document.addEventListener(requestId, responseHandler);

    const success = callRosService('/rosapi/topic_type', { topic: topic }, requestId);

    if (!success) {
        console.error(`[Topics Service] Failed to send topic type request for ${topic}`);
        document.removeEventListener(requestId, responseHandler);
        topicTypesCache[topic] = 'Error'; // Mark as failed
    }
}

/**
 * Display the list of topics in the UI
 */
function displayTopicsList() {
    const topicsList = document.getElementById('topics-list');
    if (!topicsList) return;
    
    // Create table rows for topics
    let tableContent = '';
    
    if (topicsCache.length === 0) {
        tableContent = '<tr><td colspan="2" class="has-text-centered">No topics found</td></tr>';
    } else {
        topicsCache.forEach(topic => {
            const topicType = topicTypesCache[topic] || 'Unknown';
            
            tableContent += `
                <tr>
                    <td>
                        <a href="#" class="topic-name-link" data-topic="${topic}" data-type="${topicType}">
                            ${topic}
                        </a>
                    </td>
                    <td>
                        <div class="buttons are-small">
                            <button class="button is-success topic-echo-btn" data-topic="${topic}" data-type="${topicType}">
                                <span class="icon"><i class="fas fa-eye"></i></span>
                            </button>
                        </div>
                    </td>
                </tr>
            `;
        });
    }
    
    // Update the table content
    topicsList.innerHTML = tableContent;
    
    // Add event listeners to action buttons
    setupTopicActionButtons();
}

/**
 * Set up event listeners for topic action buttons
 */
function setupTopicActionButtons() {
    // Topic name links
    const nameLinks = document.querySelectorAll('.topic-name-link');
    nameLinks.forEach(link => {
        link.addEventListener('click', function(e) {
            e.preventDefault();
            const topicName = this.getAttribute('data-topic');
            showTopicInfo(topicName);
        });
    });

    // Echo buttons
    const echoButtons = document.querySelectorAll('.topic-echo-btn');
    echoButtons.forEach(btn => {
        btn.addEventListener('click', function() {
            const topicName = this.getAttribute('data-topic');
            setupTopicEcho(topicName);
        });
    });
}

/**
 * Show topic information in a modal
 * @param {string} topicName - Name of the topic to show info for
 */
function showTopicInfo(topicName) {
    logToConsole(`Showing info for topic: ${topicName}`);
    
    // Get topic type from cache or fetch it if not available
    const topicType = topicTypesCache[topicName] || 'Unknown';
    console.log(`[Topics Service] Topic type for ${topicName}: ${topicType}`);
    
    // Create and show the modal first
    const modalHtml = `
        <div id="topic-info-modal" class="modal is-active">
            <div class="modal-background close-modal"></div>
            <div class="modal-card">
                <header class="modal-card-head">
                    <p class="modal-card-title">Topic Information: ${topicName}</p>
                    <button class="delete close-modal" aria-label="close"></button>
                </header>
                <section class="modal-card-body">
                    <table class="table is-fullwidth is-hoverable">
                        <thead>
                            <tr>
                                <th>Property</th>
                                <th>Value</th>
                            </tr>
                        </thead>
                        <tbody>
                            <tr>
                                <td><code>Type</code></td>
                                <td><code id="topic-type-value">${topicType}</code></td>
                            </tr>
                            <tr>
                                <td><code>Frequency</code></td>
                                <td><code id="topic-frequency">Measuring...</code></td>
                            </tr>
                        </tbody>
                    </table>
                    <div class="field" id="publishers-field">
                        <label class="label">Publishers</label>
                        <div class="control">
                            <div class="is-loading has-text-centered" id="publishers-loading">
                                Loading publishers...
                            </div>
                            <div id="publishers-list" class="content">
                                <!-- Publishers will be listed here -->
                            </div>
                        </div>
                    </div>
                    <div class="field" id="subscribers-field">
                        <label class="label">Subscribers</label>
                        <div class="control">
                            <div class="is-loading has-text-centered" id="subscribers-loading">
                                Loading subscribers...
                            </div>
                            <div id="subscribers-list" class="content">
                                <!-- Subscribers will be listed here -->
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
            // Stop frequency measurement if active
            stopFrequencyMeasurement();
            document.getElementById('topic-info-modal').remove();
        });
    });
    
    // Get publishers and subscribers for this topic
    queryTopicPublishers(topicName);
    queryTopicSubscribers(topicName);
    
    // If topic type is unknown, fetch it now
    if (topicType === 'Unknown') {
        console.log(`[Topics Service] Topic type not in cache for ${topicName}, fetching...`);
        getTopicType(topicName);
    }
    
    // Start frequency measurement automatically
    setTimeout(() => {
        startFrequencyMeasurement(topicName);
    }, 100);
}

/**
 * Set up topic echo functionality
 * @param {string} topicName - Name of the topic to echo
 */
function setupTopicEcho(topicName) {
    logToConsole(`Setting up echo for topic: ${topicName}`);
    
    // Create modal for topic echo
    const modalHtml = `
        <div class="modal is-active" id="topic-echo-modal">
            <div class="modal-background"></div>
            <div class="modal-card">
                <header class="modal-card-head">
                    <p class="modal-card-title">Topic Echo: ${topicName}</p>
                    <button class="delete close-modal" aria-label="close"></button>
                </header>
                <section class="modal-card-body">
                    <div class="field">
                        <div class="control">
                            <textarea class="textarea" id="echo-output" rows="15" readonly placeholder="Topic messages will appear here..."></textarea>
                        </div>
                    </div>
                </section>
                <footer class="modal-card-foot">
                    <button class="button is-success" id="start-echo-btn">Start</button>
                    <button class="button is-danger" id="stop-echo-btn" disabled>Stop</button>
                    <button class="button" id="clear-echo-btn">Clear</button>
                    <button class="button close-modal">Close</button>
                </footer>
            </div>
        </div>
    `;
    
    // Add modal to document
    document.body.insertAdjacentHTML('beforeend', modalHtml);
    
    // Set up close buttons
    const closeButtons = document.querySelectorAll('.close-modal');
    closeButtons.forEach(btn => {
        btn.addEventListener('click', function() {
            // Stop echo if active
            stopTopicEcho();
            document.getElementById('topic-echo-modal').remove();
        });
    });
    
    // Set up control buttons
    const startBtn = document.getElementById('start-echo-btn');
    const stopBtn = document.getElementById('stop-echo-btn');
    const clearBtn = document.getElementById('clear-echo-btn');
    const output = document.getElementById('echo-output');
    
    if (startBtn) {
        startBtn.addEventListener('click', function() {
            startTopicEcho(topicName);
            startBtn.disabled = true;
            stopBtn.disabled = false;
        });
    }
    
    if (stopBtn) {
        stopBtn.addEventListener('click', function() {
            stopTopicEcho();
            startBtn.disabled = false;
            stopBtn.disabled = true;
        });
    }
    
    if (clearBtn) {
        clearBtn.addEventListener('click', function() {
            if (output) {
                output.value = '';
            }
        });
    }
}

/**
 * Start echoing messages from a topic
 * @param {string} topicName - Name of the topic to echo
 */
function startTopicEcho(topicName) {
    if (!rosConnection || rosConnection.readyState !== WebSocket.OPEN) {
        showNotification('WebSocket connection is not open', 'danger');
        return;
    }
    
    // Generate a unique subscription ID
    const subscriptionId = 'echo_' + Date.now();
    topicEchoSubscription = subscriptionId;
    
    // Set up message handler
    const messageHandler = function(event) {
        const message = event.detail;
        
        // For now, this just displays the JSON representation of the message
        const output = document.getElementById('echo-output');
        if (output) {
            const timestamp = new Date().toLocaleTimeString();
            output.value += `[${timestamp}] ${JSON.stringify(message.msg, null, 2)}\n\n`;
            output.scrollTop = output.scrollHeight;
        }
    };
    
    // Start listening for messages
    document.addEventListener('topic-message', messageHandler);
    
    // Create subscription message
    const subscriptionMessage = {
        op: 'subscribe',
        id: subscriptionId,
        topic: topicName,
        type: topicTypesCache[topicName] || ''
    };
    
    // Send subscription request
    if (rosConnection && rosConnection.readyState === WebSocket.OPEN) {
        rosConnection.send(JSON.stringify(subscriptionMessage));
    } else {
        showNotification('Connection unavailable', 'danger');
        return;
    }
    
    // Store subscription info for cleanup
    currentTopicSubscription = {
        topic: topicName,
        id: subscriptionId,
        handler: messageHandler
    };
    
    logToConsole(`Started echoing messages for ${topicName}`);
}

/**
 * Stop echoing messages from a topic
 */
function stopTopicEcho() {
    if (!topicEchoSubscription) {
        return;
    }
    
    if (rosConnection && rosConnection.readyState === WebSocket.OPEN) {
        // Unsubscribe from topic
        const unsubscribeMessage = {
            op: 'unsubscribe',
            id: topicEchoSubscription,
            topic: currentTopicSubscription ? currentTopicSubscription.topic : undefined
        };
        
        // Remove event listener
        document.removeEventListener(
            'topic-message', 
            currentTopicSubscription.handler
        );
        
        rosConnection.send(JSON.stringify(unsubscribeMessage));
        
        logToConsole(`Stopped echoing messages for ${currentTopicSubscription.topic}`);
        
        // Clear subscription
        currentTopicSubscription = null;
        topicEchoSubscription = null;
    }
}

/**
 * Get subscribers and publishers for a topic
 * @param {string} topic - Topic name
 */
function getTopicConnections(topic) {
    // Get subscribers
    const id = 'get_subscribers_' + Date.now();
    const subscribersMessage = {
        op: 'call_service',
        id: id,
        service: '/rosapi/subscribers',
        args: { topic: topic }
    };
    
    try {
        rosConnection.send(JSON.stringify(subscribersMessage));
    } catch (error) {
        console.error('Error getting subscribers:', error);
    }
    
    // Get publishers
    const id2 = 'get_publishers_' + Date.now();
    const publishersMessage = {
        op: 'call_service',
        id: id2,
        service: '/rosapi/publishers',
        args: { topic: topic }
    };
    
    try {
        rosConnection.send(JSON.stringify(publishersMessage));
    } catch (error) {
        console.error('Error getting publishers:', error);
    }
}

/**
 * Handle subscribers response
 * @param {Object} response - Response data from subscribers service
 */
function handleSubscribersResponse(response) {
    if (!response || !response.values || !response.values.subscribers) {
        console.error('Invalid subscribers response:', response);
        return;
    }
    
    const topic = response.values.topic;
    const subscribers = response.values.subscribers;
    topicSubscribersCache[topic] = subscribers;
}

/**
 * Handle publishers response
 * @param {Object} response - Response data from publishers service
 */
function handlePublishersResponse(response) {
    if (!response || !response.values || !response.values.publishers) {
        console.error('Invalid publishers response:', response);
        return;
    }
    
    const topic = response.values.topic;
    const publishers = response.values.publishers;
    topicPublishersCache[topic] = publishers;
}

/**
 * Start measuring the frequency of a topic
 * @param {string} topicName - Name of the topic to measure
 */
function startFrequencyMeasurement(topicName) {
    if (frequencyMeasurementActive) {
        stopFrequencyMeasurement();
    }

    frequencyMeasurementActive = true;
    messageTimestamps = [];
    measurementStartTime = Date.now(); // Start the timer

    const freqDisplay = document.getElementById('topic-frequency');
    if (freqDisplay) {
        freqDisplay.textContent = 'Measuring...';
        if (freqDisplay.parentElement) {
            // Reset color of the line
            freqDisplay.parentElement.className = '';
        }
    }

    const checkAndSubscribe = () => {
        const topicType = topicTypesCache[topicName];
        if (topicType && topicType !== 'Error' && topicType !== 'Fetching...' && topicType !== 'Unknown') {
            subscribeForFrequency(topicName, topicType);
            return true; // Success
        }
        return false; // Not ready
    };

    if (checkAndSubscribe()) {
        return; // Already had the type, subscribed.
    }

    // If type is not available, request it and start polling.
    getTopicType(topicName);

    let attempts = 0;
    const maxAttempts = 15; // Poll for 3 seconds
    const pollInterval = setInterval(() => {
        if (!frequencyMeasurementActive) { // Stop polling if user closed modal
            clearInterval(pollInterval);
            return;
        }

        if (checkAndSubscribe()) {
            clearInterval(pollInterval);
        } else {
            attempts++;
            if (attempts >= maxAttempts) {
                clearInterval(pollInterval);
                if (freqDisplay) {
                    freqDisplay.textContent = 'Measurement failed';
                    if (freqDisplay.parentElement) {
                        freqDisplay.parentElement.className = 'has-text-danger';
                    }
                }
                console.error(`[Topics] Could not get type for ${topicName} after ${maxAttempts} attempts.`);
                frequencyMeasurementActive = false;
            }
        }
    }, 200);
}

function subscribeForFrequency(topicName, topicType) {
    if (!frequencyMeasurementActive) return;

    const freqDisplay = document.getElementById('topic-frequency');
    if (freqDisplay) {
        freqDisplay.textContent = 'Measuring...';
    }

    const subscriptionId = 'freq_' + Date.now();

    const messageHandler = function(event) {
        const message = event.detail;
        if (message && message.topic === topicName) {
            messageTimestamps.push(Date.now());
        }
    };

    document.addEventListener('topic-message', messageHandler);

    const subscriptionMessage = {
        op: 'subscribe',
        id: subscriptionId,
        topic: topicName,
        type: topicType
    };

    if (rosConnection && rosConnection.readyState === WebSocket.OPEN) {
        rosConnection.send(JSON.stringify(subscriptionMessage));
        
        currentTopicSubscription = {
            topic: topicName,
            id: subscriptionId,
            handler: messageHandler
        };

        frequencyUpdateInterval = setInterval(updateFrequencyDisplay, 500);
        logToConsole(`Subscribed to ${topicName} for frequency measurement.`);

    } else {
        if (freqDisplay) {
            freqDisplay.textContent = 'Connection failed';
        }
        logToConsole('Cannot subscribe for frequency: ROS connection not open.', 'error');
        frequencyMeasurementActive = false;
    }
}

function stopFrequencyMeasurement() {
    if (!frequencyMeasurementActive) return;

    frequencyMeasurementActive = false;

    if (frequencyUpdateInterval) {
        clearInterval(frequencyUpdateInterval);
        frequencyUpdateInterval = null;
    }

    if (currentTopicSubscription) {
        const unsubscribeMessage = {
            op: 'unsubscribe',
            id: currentTopicSubscription.id,
            topic: currentTopicSubscription.topic
        };

        document.removeEventListener('topic-message', currentTopicSubscription.handler);

        if (rosConnection && rosConnection.readyState === WebSocket.OPEN) {
            rosConnection.send(JSON.stringify(unsubscribeMessage));
        }

        currentTopicSubscription = null;
    }

    let freqDisplay = document.getElementById('topic-frequency');
    if (freqDisplay) {
        freqDisplay.textContent = '...measuring...';
    }

    messageTimestamps = [];
}

function updateFrequencyDisplay() {
    const freqDisplay = document.getElementById('topic-frequency');
    if (!frequencyMeasurementActive || !freqDisplay || !freqDisplay.parentElement) {
        return;
    }

    const freqLine = freqDisplay.parentElement;

    // Wait for the initial measurement window to pass before showing any numbers.
    if (Date.now() - measurementStartTime < MEASUREMENT_WINDOW) {
        freqDisplay.textContent = 'Measuring...';
        freqLine.className = ''; // No color during initial measurement
        return;
    }

    const cutoffTime = Date.now() - MEASUREMENT_WINDOW;
    messageTimestamps = messageTimestamps.filter(timestamp => timestamp >= cutoffTime);

    if (messageTimestamps.length === 0) {
        freqLine.className = 'has-text-danger';
        freqDisplay.textContent = 'No messages detected (0 Hz)';
        return;
    }

    if (messageTimestamps.length < 2) {
        freqLine.className = 'has-text-warning';
        freqDisplay.textContent = 'Low frequency (< 0.2 Hz)';
        return;
    }

    const windowSizeSeconds = (Date.now() - cutoffTime) / 1000;
    const frequency = messageTimestamps.length / windowSizeSeconds;

    freqLine.className = 'has-text-success';
    freqDisplay.textContent = `${frequency.toFixed(2)} Hz`;
}

function queryTopicPublishers(topicName) {
    if (!rosConnection || rosConnection.readyState !== WebSocket.OPEN) {
        document.getElementById('publishers-loading').textContent = 'Connection unavailable';
        return;
    }
    
    const requestId = `get_publishers_${topicName.replace(/\//g, '_')}_${Date.now()}`;

    // Set up one-time event listener for the response
    const responseHandler = function(event) {
        document.removeEventListener(requestId, responseHandler);
        const response = event.detail;

        const publishersList = document.getElementById('publishers-list');
        const publishersLoading = document.getElementById('publishers-loading');
        
        if (publishersList && publishersLoading) {
            publishersLoading.style.display = 'none';
            
            if (response && response.values && response.values.publishers) {
                const publishers = response.values.publishers;
                
                if (publishers.length > 0) {
                    let tagsHtml = '';
                    publishers.forEach(pub => {
                        tagsHtml += `<span class="tag is-primary is-medium has-text-white mr-1 mb-1" style="background-color: #1a73e8;">${pub}</span>`;
                    });
                    publishersList.innerHTML = `<div class="tags">${tagsHtml}</div>`;
                } else {
                    publishersList.innerHTML = '<p>No publishers found</p>';
                }
            } else {
                publishersList.innerHTML = '<p class="has-text-danger">Failed to retrieve publishers</p>';
            }
        }
    };
    
    document.addEventListener(requestId, responseHandler);
    
    // Call the ROS service
    const success = callRosService('/rosapi/publishers', { topic: topicName }, requestId);
    if (!success) {
        document.getElementById('publishers-loading').textContent = 'Request failed';
    }
}

/**
 * Query subscribers for a topic
 * @param {string} topicName - Name of the topic
 */
function queryTopicSubscribers(topicName) {
    if (!rosConnection || rosConnection.readyState !== WebSocket.OPEN) {
        document.getElementById('subscribers-loading').textContent = 'Connection unavailable';
        return;
    }
    
    // Include debugging information
    logToConsole(`Querying subscribers for topic: ${topicName}`);
    
    const requestId = `get_subscribers_${topicName.replace(/\//g, '_')}_${Date.now()}`;

    // Set up one-time event listener for the response
    const responseHandler = function(event) {
        logToConsole(`Received subscriber response for ${topicName}`);
        document.removeEventListener(requestId, responseHandler);
        const response = event.detail;
        
        // Log the full response for debugging
        console.log("Subscribers response:", response);
        
        const subscribersList = document.getElementById('subscribers-list');
        const subscribersLoading = document.getElementById('subscribers-loading');
        
        if (subscribersList && subscribersLoading) {
            subscribersLoading.style.display = 'none';
            
            if (response && response.values) {
                // Log the values for debugging
                console.log("Subscribers values:", response.values);
                
                // Some rosbridge implementations use 'subscribers' key, others use 'nodes'
                const subscribers = response.values.subscribers || response.values.nodes || [];
                
                if (subscribers && subscribers.length > 0) {
                    let tagsHtml = '';
                    subscribers.forEach(sub => {
                        tagsHtml += `<span class="tag is-info is-medium has-text-white mr-1 mb-1" style="background-color: #0d652d;">${sub}</span>`;
                    });
                    subscribersList.innerHTML = `<div class="tags">${tagsHtml}</div>`;
                } else {
                    subscribersList.innerHTML = '<p>No subscribers found</p>';
                    logToConsole(`No subscribers found for ${topicName} (response format might be wrong)`, 'warn');
                }
            } else {
                subscribersList.innerHTML = '<p class="has-text-danger">Failed to retrieve subscribers</p>';
                logToConsole(`Failed to retrieve subscribers for ${topicName}`, 'error');
            }
        }
    };
    
    document.addEventListener(requestId, responseHandler);
    
    // Call the ROS service
    const success = callRosService('/rosapi/subscribers', { topic: topicName }, requestId);
    if (!success) {
        document.getElementById('subscribers-loading').textContent = 'Request failed';
        logToConsole(`Failed to send subscribers request for ${topicName}`, 'error');
    }
}
