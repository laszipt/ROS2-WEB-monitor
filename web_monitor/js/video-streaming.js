/**
 * Video Streaming Service for OAK Detection Node
 * Handles video stream display and controls
 */

class VideoStreamingService {
    constructor() {
        // Use current host instead of hardcoded localhost
        const currentHost = window.location.hostname || 'localhost';
        this.streamUrl = `http://${currentHost}:5001/oak_detection_image`;
        this.isStreaming = false;
        this.videoElement = null;
        this.statusElement = null;
        this.placeholderElement = null;
        this.streamInfoElement = null;
        
        // Button elements
        this.startBtn = null;
        this.stopBtn = null;
        this.refreshBtn = null;
        
        console.log(`Video Streaming Service - Stream URL: ${this.streamUrl}`);
        this.init();
    }
    
    init() {
        // Get DOM elements
        this.videoElement = document.getElementById('videoStream');
        this.statusElement = document.getElementById('streamStatus');
        this.placeholderElement = document.getElementById('videoPlaceholder');
        this.streamInfoElement = document.getElementById('streamInfo');
        
        this.startBtn = document.getElementById('startStreamBtn');
        this.stopBtn = document.getElementById('stopStreamBtn');
        this.refreshBtn = document.getElementById('refreshStreamBtn');
        
        // Add event listeners
        if (this.startBtn) {
            this.startBtn.addEventListener('click', () => this.startStream());
        }
        
        if (this.stopBtn) {
            this.stopBtn.addEventListener('click', () => this.stopStream());
        }
        
        if (this.refreshBtn) {
            this.refreshBtn.addEventListener('click', () => this.refreshStream());
        }
        
        // Add error handling for video element
        if (this.videoElement) {
            this.videoElement.addEventListener('load', () => this.onStreamLoad());
            this.videoElement.addEventListener('error', (e) => this.onStreamError(e));
        }
        
        console.log('Video Streaming Service initialized');
        
        // Test connection availability on initialization
        this.testConnection();
    }
    
    startStream() {
        if (this.isStreaming) {
            console.log('Stream already active');
            return;
        }
        
        console.log('Starting video stream from:', this.streamUrl);
        
        // Update UI state
        this.updateStatus('Connecting...', 'is-warning');
        this.startBtn.disabled = true;
        this.stopBtn.disabled = false;
        
        // Set video source and show video element
        if (this.videoElement) {
            // For MJPEG streams, we need to handle them differently
            const streamUrlWithTimestamp = this.streamUrl + '?t=' + Date.now();
            console.log('Setting video source to:', streamUrlWithTimestamp);
            
            // Clear any previous source
            this.videoElement.src = '';
            
            // Set up the image element for MJPEG stream
            this.videoElement.onload = () => this.onStreamLoad();
            this.videoElement.onerror = (e) => this.onStreamError(e);
            
            // Set the source - this should work for MJPEG streams
            this.videoElement.src = streamUrlWithTimestamp;
            this.videoElement.style.display = 'block';
        }
        
        // Hide placeholder and show stream info
        if (this.placeholderElement) {
            this.placeholderElement.style.display = 'none';
        }
        
        if (this.streamInfoElement) {
            this.streamInfoElement.style.display = 'block';
        }
        
        this.isStreaming = true;
        
        // Set a timeout to check if stream loaded successfully
        setTimeout(() => {
            if (this.isStreaming && this.videoElement) {
                // For MJPEG streams, check if the image has started loading
                if (this.videoElement.naturalWidth === 0 && this.videoElement.naturalHeight === 0) {
                    console.warn('Stream did not load within timeout period');
                    this.onStreamError(new Error('Stream timeout - no image data received'));
                } else {
                    console.log('Stream appears to be loading, dimensions:', this.videoElement.naturalWidth, 'x', this.videoElement.naturalHeight);
                }
            }
        }, 5000); // Reduced back to 5 seconds for initial check
    }
    
    stopStream() {
        if (!this.isStreaming) {
            console.log('Stream already stopped');
            return;
        }
        
        console.log('Stopping video stream');
        
        // Update UI state
        this.updateStatus('Disconnected', 'is-warning');
        this.startBtn.disabled = false;
        this.stopBtn.disabled = true;
        
        // Hide video element and show placeholder
        if (this.videoElement) {
            this.videoElement.src = '';
            this.videoElement.style.display = 'none';
        }
        
        if (this.placeholderElement) {
            this.placeholderElement.style.display = 'block';
        }
        
        if (this.streamInfoElement) {
            this.streamInfoElement.style.display = 'none';
        }
        
        this.isStreaming = false;
    }
    
    refreshStream() {
        console.log('Refreshing video stream');
        
        if (this.isStreaming) {
            // Restart the stream
            this.stopStream();
            setTimeout(() => {
                this.startStream();
            }, 500);
        } else {
            // Just try to start if not streaming
            this.startStream();
        }
    }
    
    onStreamLoad() {
        console.log('Video stream loaded successfully');
        this.updateStatus('Connected', 'is-success');
        
        // Show notification
        if (typeof showNotification === 'function') {
            showNotification('Video stream connected successfully', 'success', 3000);
        }
    }
    
    onStreamError(e) {
        console.error('Video stream failed to load:', e);
        this.updateStatus('Connection Failed', 'is-danger');
        
        // Reset UI state
        this.startBtn.disabled = false;
        this.stopBtn.disabled = true;
        
        if (this.videoElement) {
            this.videoElement.style.display = 'none';
        }
        
        if (this.placeholderElement) {
            this.placeholderElement.style.display = 'block';
        }
        
        if (this.streamInfoElement) {
            this.streamInfoElement.style.display = 'none';
        }
        
        this.isStreaming = false;
        
        // Show error notification
        if (typeof showNotification === 'function') {
            showNotification('Failed to connect to video stream. Make sure the OAK detection node is running.', 'error', 5000);
        }
    }
    
    async testConnection() {
        console.log('Testing connection to OAK detection stream...');
        try {
            const response = await fetch(this.streamUrl, { 
                method: 'HEAD',
                mode: 'no-cors',
                timeout: 5000
            });
            console.log('Stream endpoint is reachable');
            this.updateStatus('Ready', 'is-info');
        } catch (error) {
            console.warn('Stream endpoint not reachable:', error.message);
            this.updateStatus('Service Unavailable', 'is-warning');
            
            // Show helpful notification
            if (typeof showNotification === 'function') {
                showNotification('OAK detection stream not available. Make sure the oak_detection_node is running.', 'warning', 5000);
            }
        }
    }
    
    updateStatus(text, className) {
        if (this.statusElement) {
            this.statusElement.textContent = text;
            this.statusElement.className = 'tag ' + className;
        }
    }
    
    // Method to check if OAK detection node is available
    async checkStreamAvailability() {
        try {
            const response = await fetch(this.streamUrl, { 
                method: 'HEAD',
                mode: 'no-cors' // This will help with CORS issues
            });
            return true;
        } catch (error) {
            console.log('Stream not available:', error.message);
            return false;
        }
    }
}

// Global instance
let videoStreamingService = null;

// Initialize when DOM is loaded
document.addEventListener('DOMContentLoaded', function() {
    console.log('Video Streaming: DOM loaded, checking for video elements...');
    
    // Check if video elements exist
    const videoElement = document.getElementById('videoStream');
    const startBtn = document.getElementById('startStreamBtn');
    
    if (videoElement || startBtn) {
        console.log('Video Streaming: Elements found, initializing service...');
        videoStreamingService = new VideoStreamingService();
    } else {
        console.log('Video Streaming: Elements not found, will initialize when section is accessed');
        
        // Set up a mutation observer to watch for when the video section becomes available
        const observer = new MutationObserver(function(mutations) {
            mutations.forEach(function(mutation) {
                if (mutation.type === 'childList') {
                    const videoElement = document.getElementById('videoStream');
                    const startBtn = document.getElementById('startStreamBtn');
                    
                    if ((videoElement || startBtn) && !videoStreamingService) {
                        console.log('Video Streaming: Elements now available, initializing service...');
                        videoStreamingService = new VideoStreamingService();
                        observer.disconnect(); // Stop observing once initialized
                    }
                }
            });
        });
        
        // Start observing
        observer.observe(document.body, {
            childList: true,
            subtree: true
        });
    }
});

// Also try to initialize when the video section becomes active
document.addEventListener('click', function(e) {
    if (e.target && e.target.getAttribute('data-target') === 'video-section') {
        console.log('Video Streaming: Video section clicked, ensuring service is initialized...');
        setTimeout(() => {
            if (!videoStreamingService) {
                const videoElement = document.getElementById('videoStream');
                const startBtn = document.getElementById('startStreamBtn');
                
                if (videoElement || startBtn) {
                    console.log('Video Streaming: Initializing service after section switch...');
                    videoStreamingService = new VideoStreamingService();
                }
            }
        }, 100); // Small delay to ensure elements are rendered
    }
});

// Export for use in other scripts
if (typeof module !== 'undefined' && module.exports) {
    module.exports = VideoStreamingService;
}
