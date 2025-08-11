/**
 * ui-controller.js
 * Handles UI interactions and menu navigation
 */

document.addEventListener('DOMContentLoaded', function() {
    initializeMenu();
    initializeTabs();
});

/**
 * Initialize the sidebar menu functionality
 */
function initializeMenu() {
    // Find all menu items
    const menuItems = document.querySelectorAll('.menu-item');
    
    // Add click event listener to each menu item
    menuItems.forEach(item => {
        item.addEventListener('click', function(e) {
            e.preventDefault();
            
            // Get the target section ID
            const targetSectionId = this.getAttribute('data-target');
            
            // Switch to the selected section
            switchSection(targetSectionId);
            
            // Update menu active state
            updateMenuActiveState(this);
            
            // Log the section change
            logToConsole(`Navigated to ${targetSectionId}`);
        });
    });
    
    // Log successful initialization
    logToConsole("Menu initialized successfully");
}

/**
 * Switch to the selected content section
 * @param {string} sectionId - ID of the section to display
 */
function switchSection(sectionId) {
    // Find all section containers
    const sections = document.querySelectorAll('.section-container');
    
    // Find currently active section first, so we can trigger deactivation event
    const activeSection = document.querySelector('.section-container.is-active');
    if (activeSection) {
        const activeSectionId = activeSection.id;
        logToConsole(`Deactivating section: ${activeSectionId}`, 'info');
        console.log(`[UI Controller] Deactivating section: ${activeSectionId}`);
        
        // Trigger deactivation event for the previous section
        const eventData = { sectionId: activeSectionId };
        console.log(`[UI Controller] Dispatching section-deactivated event with data:`, eventData);
        
        // Make sure the triggerEvent function exists
        if (typeof triggerEvent === 'function') {
            triggerEvent('section-deactivated', eventData);
        } else {
            // Fallback if triggerEvent function is not found
            console.error("[UI Controller] triggerEvent function not found, using direct event dispatch");
            const event = new CustomEvent('section-deactivated', { detail: eventData });
            document.dispatchEvent(event);
            console.log("[UI Controller] section-deactivated event dispatched directly");
        }
        
        // Remove active class
        activeSection.classList.remove('is-active');
    }
    
    // Hide all other sections
    sections.forEach(section => {
        if (section !== activeSection) { // Skip the one we just processed
            section.classList.remove('is-active');
        }
    });
    
    // Show the selected section
    const targetSection = document.getElementById(sectionId);
    if (targetSection) {
        targetSection.classList.add('is-active');
        
        // Trigger section-specific initialization if needed
        triggerSectionInitialization(sectionId);
    } else {
        logToConsole(`Error: Section ${sectionId} not found`, 'error');
    }
}

/**
 * Update active state of menu items
 * @param {Element} activeItem - The currently active menu item
 */
function updateMenuActiveState(activeItem) {
    // Remove active class from all menu items
    const menuItems = document.querySelectorAll('.menu-item');
    menuItems.forEach(item => {
        item.classList.remove('is-active');
    });
    
    // Add active class to the selected item
    activeItem.classList.add('is-active');
}

/**
 * Trigger section-specific initialization when a section is displayed
 * @param {string} sectionId - ID of the section being displayed
 */
function triggerSectionInitialization(sectionId) {
    // Log which section we're initializing
    logToConsole(`Initializing section: ${sectionId}`, 'info');
    
    // Helper function to safely call a function if it exists
    const safeCall = (fn, ...args) => {
        if (typeof window[fn] === 'function') {
            window[fn](...args);
            return true;
        } 
        return false;
    };
    
    // Trigger appropriate initialization based on section ID
    switch (sectionId) {
        case 'nodes-section':
            triggerEvent('section-nodes-activated');
            if (typeof refreshNodesList === 'function') {
                refreshNodesList();
            } else {
                logToConsole('refreshNodesList function not available', 'warn');
            }
            break;
        case 'topics-section':
            triggerEvent('section-topics-activated');
            if (typeof refreshTopicsList === 'function') {
                refreshTopicsList();
            } else {
                logToConsole('refreshTopicsList function not available', 'warn');
            }
            break;
        case 'diagnostics-section':
            triggerEvent('section-diagnostics-activated');
            // Diagnostics initialization will be implemented later
            break;
        case 'parameters-section':
            triggerEvent('section-parameters-activated');
            if (typeof refreshParameterFiles === 'function') {
                refreshParameterFiles();
            }
            break;
        case 'logs-section':
            triggerEvent('section-logs-activated');
            break;
        case 'system-section':
            triggerEvent('section-system-activated');
            if (typeof setupGaugeCharts === 'function') {
                setupGaugeCharts();
            } else {
                logToConsole('setupGaugeCharts function not available', 'warn');
            }
            break;
        default:
            logToConsole(`No specific initialization for section: ${sectionId}`, 'debug');
    }
}

/**
 * Show a notification message to the user
 * @param {string} message - Message to show
 * @param {string} type - Message type (info, success, warning, danger)
 * @param {number} duration - Duration in milliseconds (0 for permanent)
 */
function showNotification(message, type = 'info', duration = 5000) {
    // Create notification element
    const notification = document.createElement('div');
    notification.className = `notification is-${type}`;
    notification.innerHTML = `
        <button class="delete"></button>
        ${message}
    `;
    
    // Add to the document
    const notificationContainer = document.getElementById('notification-container') || document.body;
    notificationContainer.appendChild(notification);
    
    // Add click handler to delete button
    const deleteBtn = notification.querySelector('.delete');
    if (deleteBtn) {
        deleteBtn.addEventListener('click', () => {
            notification.remove();
        });
    }
    
    // Auto-remove after duration (if not permanent)
    if (duration > 0) {
        setTimeout(() => {
            if (notification.parentNode) {
                notification.remove();
            }
        }, duration);
    }
}

/**
 * Show loading spinner in a container
 * @param {string} containerId - ID of the container element
 * @param {string} message - Optional loading message
 */
function showLoading(containerId, message = 'Loading...') {
    const container = document.getElementById(containerId);
    if (!container) return;
    
    // Clear container
    container.innerHTML = `
        <div class="has-text-centered p-4">
            <div class="loader is-loading mb-3" style="height: 50px; width: 50px; margin: 0 auto;"></div>
            <p>${message}</p>
        </div>
    `;
}

/**
 * Hide loading spinner and restore or update content
 * @param {string} containerId - ID of the container element
 * @param {string} content - HTML content to display
 */
function hideLoading(containerId, content = '') {
    const container = document.getElementById(containerId);
    if (!container) return;
    
    // Update container content
    container.innerHTML = content;
}

/**
 * Initialize tab functionality across the dashboard
 */
function initializeTabs() {
    // Find all tab links
    const tabLinks = document.querySelectorAll('.tabs a');
    
    // Add click event listener to each tab link
    tabLinks.forEach(link => {
        link.addEventListener('click', function(e) {
            e.preventDefault();
            
            // Get the target tab content ID
            const targetTabId = this.getAttribute('data-target');
            
            // Get the parent tabs group
            const tabsGroup = this.closest('.tabs');
            if (!tabsGroup) return;
            
            // Find the parent tab content container
            const tabContentContainer = tabsGroup.nextElementSibling;
            if (!tabContentContainer || !tabContentContainer.classList.contains('tab-content')) {
                return;
            }
            
            // Find all tab content elements in this container
            const tabContents = tabContentContainer.querySelectorAll('[id]');
            
            // Hide all tab contents
            tabContents.forEach(content => {
                content.classList.add('is-hidden');
                content.classList.remove('is-active');
            });
            
            // Show the selected tab content
            const targetTabContent = document.getElementById(targetTabId);
            if (targetTabContent) {
                targetTabContent.classList.remove('is-hidden');
                targetTabContent.classList.add('is-active');
            }
            
            // Update active state of tab links
            const allLinks = tabsGroup.querySelectorAll('a');
            allLinks.forEach(item => {
                item.parentElement.classList.remove('is-active');
            });
            
            // Set this link's parent (the li element) as active
            this.parentElement.classList.add('is-active');
            
            // Trigger tab change event
            triggerEvent('tab-changed', { 
                tabId: targetTabId
            });
            
            logToConsole(`Switched to tab: ${targetTabId}`);
        });
    });
    
    logToConsole("Tab functionality initialized");
}

/**
 * Show error message in a container
 * @param {string} containerId - ID of the container element
 * @param {string} message - Error message to display
 */
function showError(containerId, message) {
    const container = document.getElementById(containerId);
    if (!container) return;
    
    // Display error message
    container.innerHTML = `
        <div class="notification is-danger">
            <p><strong>Error:</strong> ${message}</p>
            <p class="mt-2">
                <button class="button is-small is-danger is-light retry-button">
                    <span class="icon"><i class="fas fa-sync"></i></span>
                    <span>Retry</span>
                </button>
            </p>
        </div>
    `;
    
    // Add retry button functionality
    const retryButton = container.querySelector('.retry-button');
    if (retryButton) {
        retryButton.addEventListener('click', function() {
            // Determine what to retry based on container ID
            if (containerId === 'nodes-list') {
                refreshNodesList();
            } else if (containerId === 'topics-list') {
                refreshTopicsList();
            } else {
                // Generic retry event
                triggerEvent('retry-request', containerId);
            }
        });
    }
}

// Parameters logic moved to parameters-service.js
