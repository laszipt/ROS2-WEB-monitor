/**
 * System Monitor script for ROS2 Web Monitor
 * 
 * Handles system resource monitoring and visualization using ROS diagnostics
 */

// Initialize the system monitoring when document is ready
document.addEventListener('DOMContentLoaded', function() {
    // We will now handle initialization via our sequential loader
    // initSystemMonitoring();
    
    // Set up global section deactivation listener right away to ensure it's registered early
    document.addEventListener('section-deactivated', handleSectionDeactivation);
});

// Gauge chart objects
let cpuGaugeChart = null;
let ramGaugeChart = null;
let diskGaugeChart = null;

// System stats cache
let systemStats = {
    cpuAvg: 0,
    ramPercent: 0,
    diskPercent: 0,
    lastUpdate: null
};

// Track subscription status
let diagnosticsSubscribed = false;
let diagnosticsSubscriptionId = null;

// Global debugging counter
let diagnosticDebugCount = 0;

// Handle section deactivation event
function handleSectionDeactivation(event) {
    const deactivatedSection = event.detail && event.detail.sectionId;
    
    // Check if it's the system section that was deactivated
    if (deactivatedSection === 'system-section') {
        unsubscribeFromSystemStats();
    }
}

// Initialize system monitoring visualization
function initSystemMonitoring() {
    // Set up event listeners
    document.addEventListener('section-system-activated', function() {
        if (!cpuGaugeChart || !ramGaugeChart || !diskGaugeChart) {
            setupGaugeCharts();
        }
        updateGaugeCharts();
        
        // Subscribe to diagnostics only when system section is active
        subscribeToSystemStats();
    });
    
    // Make sure our deactivation handler is registered
    document.removeEventListener('section-deactivated', handleSectionDeactivation); // Remove any duplicate
    document.addEventListener('section-deactivated', handleSectionDeactivation);
    
    document.addEventListener('ros-connected', function() {
        // Only subscribe if system section is currently active
        const systemSection = document.getElementById('system-section');
        if (systemSection && systemSection.classList.contains('is-active')) {
            subscribeToSystemStats();
        }
    });
    
    // Set up gauge charts if not already done
    if (!cpuGaugeChart || !ramGaugeChart || !diskGaugeChart) {
        setupGaugeCharts();
    }
}

// Set up gauge charts for CPU, RAM, and Disk
function setupGaugeCharts() {
    // Check if Chart is available
    if (typeof Chart === 'undefined') {
        console.error('Chart.js library not loaded yet. Charts will not render.');
        return;
    }
    
    // Get chart canvas elements
    const cpuCanvas = document.getElementById('cpuChart');
    const ramCanvas = document.getElementById('ramChart');
    const diskCanvas = document.getElementById('diskChart');
    
    if (!cpuCanvas || !ramCanvas || !diskCanvas) {
        console.log('Chart canvas elements not found in DOM. Are you in the system section?');
        return;
    }
    
    // Gauge chart configuration
    try {
        // Create CPU gauge chart
        cpuGaugeChart = new Chart(cpuCanvas, {
            type: 'doughnut',
            data: {
                datasets: [{
                    data: [systemStats.cpuAvg || 0, 100 - (systemStats.cpuAvg || 0)],
                    backgroundColor: [
                        'rgba(255, 99, 132, 0.8)',
                        'rgba(240, 240, 240, 0.5)'
                    ],
                    borderWidth: 0,
                    circumference: 180,
                    rotation: 270
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                cutout: '70%',
                plugins: {
                    legend: {
                        display: false
                    },
                    tooltip: {
                        enabled: false
                    },
                    title: {
                        display: true,
                        text: `CPU: ${systemStats.cpuAvg || 0}%`,
                        position: 'bottom',
                        font: {
                            size: 16,
                            weight: 'bold'
                        }
                    }
                }
            }
        });
        
        // Create RAM gauge chart
        ramGaugeChart = new Chart(ramCanvas, {
            type: 'doughnut',
            data: {
                datasets: [{
                    data: [systemStats.ramPercent || 0, 100 - (systemStats.ramPercent || 0)],
                    backgroundColor: [
                        'rgba(54, 162, 235, 0.8)',
                        'rgba(240, 240, 240, 0.5)'
                    ],
                    borderWidth: 0,
                    circumference: 180,
                    rotation: 270
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                cutout: '70%',
                plugins: {
                    legend: {
                        display: false
                    },
                    tooltip: {
                        enabled: false
                    },
                    title: {
                        display: true,
                        text: `RAM: ${systemStats.ramPercent || 0}%`,
                        position: 'bottom',
                        font: {
                            size: 16,
                            weight: 'bold'
                        }
                    }
                }
            }
        });
        
        // Create Disk gauge chart
        diskGaugeChart = new Chart(diskCanvas, {
            type: 'doughnut',
            data: {
                datasets: [{
                    data: [systemStats.diskPercent || 0, 100 - (systemStats.diskPercent || 0)],
                    backgroundColor: [
                        'rgba(255, 159, 64, 0.8)',
                        'rgba(240, 240, 240, 0.5)'
                    ],
                    borderWidth: 0,
                    circumference: 180,
                    rotation: 270
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                cutout: '70%',
                plugins: {
                    legend: {
                        display: false
                    },
                    tooltip: {
                        enabled: false
                    },
                    title: {
                        display: true,
                        text: `Disk: ${systemStats.diskPercent || 0}%`,
                        position: 'bottom',
                        font: {
                            size: 16,
                            weight: 'bold'
                        }
                    }
                }
            }
        });
    } catch (error) {
        console.error('Error creating system gauge charts:', error);
    }
}

// Subscribe to ROS diagnostics for system statistics
function subscribeToSystemStats() {
    // Don't subscribe again if already subscribed
    if (diagnosticsSubscribed) {
        return true;
    }
    
    if (!rosConnection || rosConnection.readyState !== WebSocket.OPEN) {
        console.warn('[System Monitor] Cannot subscribe to diagnostics: WebSocket connection is not open');
        return false;
    }
    
    // Subscribe to the diagnostics topic
    diagnosticsSubscriptionId = 'subscribe_diagnostics_' + Date.now();
    const subscribeMsg = {
        op: 'subscribe',
        topic: '/diagnostics',
        id: diagnosticsSubscriptionId
    };
    
    try {
        rosConnection.send(JSON.stringify(subscribeMsg));
        diagnosticsSubscribed = true;
        logToConsole('Subscribed to /diagnostics topic for system monitoring');
        
        // Handle incoming messages
        document.removeEventListener('topic-message', handleDiagnosticsMessage); // Remove any existing handler first
        document.addEventListener('topic-message', handleDiagnosticsMessage);
        
        return true;
    } catch (error) {
        console.error('[System Monitor] Failed to subscribe to /diagnostics:', error);
        logToConsole('Failed to subscribe to /diagnostics: ' + error.message, 'error');
        return false;
    }
};

// Unsubscribe from diagnostics when section is not active
function unsubscribeFromSystemStats() {
    if (!diagnosticsSubscribed || !diagnosticsSubscriptionId) {
        return;
    }
    
    if (!rosConnection || rosConnection.readyState !== WebSocket.OPEN) {
        diagnosticsSubscribed = false;
        document.removeEventListener('topic-message', handleDiagnosticsMessage);
        return;
    }
    
    try {
        // Send unsubscribe message
        const unsubscribeMsg = {
            op: 'unsubscribe',
            topic: '/diagnostics',
            id: diagnosticsSubscriptionId
        };
        
        rosConnection.send(JSON.stringify(unsubscribeMsg));
        logToConsole('Unsubscribed from /diagnostics topic');
        
        // Remove event listener to stop processing messages
        document.removeEventListener('topic-message', handleDiagnosticsMessage);
        
        // Update subscription status
        diagnosticsSubscribed = false;
        diagnosticsSubscriptionId = null;
    } catch (error) {
        console.error('[System Monitor] Error unsubscribing from /diagnostics:', error);
    }
};

// Handle diagnostics messages
function handleDiagnosticsMessage(event) {
    // Quick check if we're still supposed to be subscribed
    if (!diagnosticsSubscribed) {
        return;
    }
    
    const message = event.detail;
    
    // Check if this is a diagnostics message
    if (message.topic !== '/diagnostics') {
        return;
    }
    
    // TEMPORARY DEBUG: Log the first few messages to see their structure
    // Remove this after debugging
    if (diagnosticDebugCount < 3) {
        console.log("=== DIAGNOSTIC MESSAGE STRUCTURE ===");
        console.log(JSON.stringify(message.msg, null, 2));
        diagnosticDebugCount++;
    }
    
    // Process the message if it contains system statistics
    try {
        const diagnostics = message.msg;
        
        // Look for system statistics in the diagnostics message
        if (diagnostics && diagnostics.status) {
            let foundStats = false;
            
            for (const status of diagnostics.status) {
                // TEMPORARY DEBUG: Log all status names to find relevant ones
                // console.log("Status name:", status.name);
                
                // Extract system stats from values
                let cpuValue = null;
                let ramValue = null;
                let diskValue = null;
                
                // Check if this status has system resource information
                if (status.values) {
                    // Look for CPU, RAM, and disk data in this status
                    const valueMap = new Map();
                    for (const keyValue of status.values) {
                        // Handle keys case-insensitively and store both original and lowercase 
                        const key = keyValue.key.toLowerCase();
                        const originalKey = keyValue.key;
                        const value = keyValue.value;
                        valueMap.set(key, value);
                        valueMap.set(originalKey, value); // Also store with original case
                        
                        // DEBUG: Log relevant metrics we find
                        if (key.includes('cpu') || key.includes('ram') || 
                            key.includes('memory') || key.includes('disk')) {
                            console.log(`Found metric: ${status.name} -> ${keyValue.key} = ${keyValue.value}`);
                        }
                    }
                    
                    // CPU usage parsing - first try direct match for "CPU AVG"
                    if (valueMap.has('CPU AVG')) {
                        let parsed = parseFloat(valueMap.get('CPU AVG'));
                        if (!isNaN(parsed)) {
                            // Assume it's already a percentage
                            cpuValue = parsed;
                            console.log(`Parsed CPU AVG: ${valueMap.get('CPU AVG')} -> ${cpuValue}%`);
                            foundStats = true;
                        }
                    }
                    
                    // If direct match failed, try lowercase keys
                    if (cpuValue === null) {
                        for (const [key, value] of valueMap.entries()) {
                            const lowerKey = key.toLowerCase();
                            if (lowerKey.includes('cpu') && 
                               (lowerKey.includes('avg') || lowerKey.includes('usage') || 
                                lowerKey.includes('percent') || lowerKey.includes('%'))) {
                                // Try to parse as a number
                                let parsed = parseFloat(value);
                                
                                // Check if it could be a string percentage
                                if (!isNaN(parsed)) {
                                    // If it's over 1.0 and less than or equal to 100, assume it's already a percentage
                                    if (parsed > 1.0 && parsed <= 100) {
                                        cpuValue = parsed;
                                    } 
                                    // If it's a decimal between 0 and 1, convert to percentage
                                    else if (parsed >= 0 && parsed <= 1.0) {
                                        cpuValue = parsed * 100;
                                    }
                                    
                                    // Only use if we got a valid percentage
                                    if (cpuValue !== null) {
                                        console.log(`Parsed CPU usage: ${value} -> ${cpuValue}%`);
                                        foundStats = true;
                                    }
                                }
                            }
                        }
                    }
                    
                    // RAM usage parsing
                    for (const [key, value] of valueMap.entries()) {
                        const lowerKey = key.toLowerCase();
                        if ((lowerKey.includes('ram') || lowerKey.includes('memory')) && 
                            (lowerKey.includes('usage') || lowerKey.includes('used') || 
                             lowerKey.includes('percent') || lowerKey.includes('%'))) {
                            
                            let parsed = parseFloat(value);
                            
                            // Handle percentage formats
                            if (!isNaN(parsed)) {
                                // If it's over 1.0 and less than or equal to 100, assume it's already a percentage
                                if (parsed > 1.0 && parsed <= 100) {
                                    ramValue = parsed;
                                } 
                                // If it's a decimal between 0 and 1, convert to percentage
                                else if (parsed >= 0 && parsed <= 1.0) {
                                    ramValue = parsed * 100;
                                }
                                
                                // Only use if we got a valid percentage
                                if (ramValue !== null) {
                                    console.log(`Parsed RAM usage: ${value} -> ${ramValue}%`);
                                    foundStats = true;
                                }
                            }
                        }
                    }
                    
                    // Disk usage parsing
                    for (const [key, value] of valueMap.entries()) {
                        const lowerKey = key.toLowerCase();
                        if (lowerKey.includes('disk') && 
                           (lowerKey.includes('usage') || lowerKey.includes('used') || 
                            lowerKey.includes('percent') || lowerKey.includes('%'))) {
                            
                            let parsed = parseFloat(value);
                            
                            // Handle percentage formats
                            if (!isNaN(parsed)) {
                                // If it's over 1.0 and less than or equal to 100, assume it's already a percentage
                                if (parsed > 1.0 && parsed <= 100) {
                                    diskValue = parsed;
                                } 
                                // If it's a decimal between 0 and 1, convert to percentage
                                else if (parsed >= 0 && parsed <= 1.0) {
                                    diskValue = parsed * 100;
                                }
                                
                                // Only use if we got a valid percentage
                                if (diskValue !== null) {
                                    console.log(`Parsed Disk usage: ${value} -> ${diskValue}%`);
                                    foundStats = true;
                                }
                            }
                        }
                    }
                }
                
                // Update system stats if found
                if (cpuValue !== null) {
                    systemStats.cpuAvg = Math.min(100, Math.max(0, cpuValue)); // Ensure 0-100 range
                }
                
                if (ramValue !== null) {
                    systemStats.ramPercent = Math.min(100, Math.max(0, ramValue)); // Ensure 0-100 range
                }
                
                if (diskValue !== null) {
                    systemStats.diskPercent = Math.min(100, Math.max(0, diskValue)); // Ensure 0-100 range
                }
            }
            
            if (foundStats) {
                // Update last update time
                systemStats.lastUpdate = new Date();
                console.log(`Updated stats: CPU=${systemStats.cpuAvg.toFixed(1)}%, RAM=${systemStats.ramPercent.toFixed(1)}%, Disk=${systemStats.diskPercent.toFixed(1)}%`);
                
                // Update charts with new data
                updateGaugeCharts();
            }
        }
    } catch (error) {
        console.error('[System Monitor] Error processing diagnostics message:', error);
    }
};

// For debugging diagnostics messages
function logDiagnosticsMessage(message) {
    if (!message || !message.msg || !message.msg.status) return;
    
    for (const status of message.msg.status) {
        if (status.values && status.values.length > 0) {
            console.log(`Diagnostic from: ${status.name}`);
            for (const item of status.values) {
                console.log(`    ${item.key}: ${item.value}`);
            }
            console.log('---');
        }
    }
}

// Update the gauge charts with current system stats
function updateGaugeCharts() {
    if (!cpuGaugeChart || !ramGaugeChart || !diskGaugeChart) {
        console.log('Cannot update charts - charts not initialized');
        return;
    }
    
    try {
        // Update CPU gauge
        cpuGaugeChart.data.datasets[0].data[0] = systemStats.cpuAvg;
        cpuGaugeChart.data.datasets[0].data[1] = 100 - systemStats.cpuAvg;
        cpuGaugeChart.options.plugins.title.text = `CPU: ${systemStats.cpuAvg.toFixed(1)}%`;
        cpuGaugeChart.update();
        
        // Update RAM gauge
        ramGaugeChart.data.datasets[0].data[0] = systemStats.ramPercent;
        ramGaugeChart.data.datasets[0].data[1] = 100 - systemStats.ramPercent;
        ramGaugeChart.options.plugins.title.text = `RAM: ${systemStats.ramPercent.toFixed(1)}%`;
        ramGaugeChart.update();
        
        // Update Disk gauge
        diskGaugeChart.data.datasets[0].data[0] = systemStats.diskPercent;
        diskGaugeChart.data.datasets[0].data[1] = 100 - systemStats.diskPercent;
        diskGaugeChart.options.plugins.title.text = `Disk: ${systemStats.diskPercent.toFixed(1)}%`;
        diskGaugeChart.update();
    } catch (error) {
        console.error('Error updating system gauge charts:', error);
    }
};

// Export key functions to global scope for proper initialization
// These exports MUST be at the end of the file after all functions are defined
window.initSystemMonitoring = initSystemMonitoring;
window.setupGaugeCharts = setupGaugeCharts;
window.subscribeToSystemStats = subscribeToSystemStats;
window.unsubscribeFromSystemStats = unsubscribeFromSystemStats;
window.updateGaugeCharts = updateGaugeCharts;
