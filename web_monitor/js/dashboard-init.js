/**
 * dashboard-init.js
 * Main initialization script for ROS2 Web Monitor
 */

// Wait for DOM to be fully loaded
document.addEventListener('DOMContentLoaded', function() {
    initializeDashboard();
});

/**
 * Initialize the dashboard
 */
function initializeDashboard() {
    logToConsole('Initializing ROS2 Web Monitor Dashboard');
    
    // Initialize ROS connection
    initRosConnection();
    
    // Add a small delay before trying to connect (to ensure all handlers are registered)
    setTimeout(function() {
        // Check for notification container, create if missing
        if (!document.getElementById('notification-container')) {
            const container = document.createElement('div');
            container.id = 'notification-container';
            container.className = 'notification-container';
            container.style.position = 'fixed';
            container.style.top = '10px';
            container.style.right = '10px';
            container.style.zIndex = '999';
            document.body.appendChild(container);
        }
        
        // Show welcome notification
        showNotification('Dashboard initialized. Connecting to ROS...', 'info', 3000);
    }, 500);
}

/**
 * Hook for global keyboard shortcuts
 */
document.addEventListener('keydown', function(event) {
    // Ctrl+Alt+R to reconnect
    if (event.ctrlKey && event.altKey && event.key === 'r') {
        event.preventDefault();
        connectToROS();
        showNotification('Reconnecting to ROS Bridge...', 'info');
    }
});
