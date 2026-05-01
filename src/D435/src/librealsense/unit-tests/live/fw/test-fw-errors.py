# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

# test:device each(D400*)
# test:device each(D500*)
# test:donotrun:!nightly

import pyrealsense2 as rs
from rspy import test, log
from rspy.stopwatch import Stopwatch
import pyrsutils as rsutils
import threading

# This test monitors firmware error notifications during streaming to ensure hardware stability

STREAMING_DURATION_SECONDS = 10  # Duration to stream and monitor for FW errors
ERROR_TOLERANCE = 0  # No firmware errors should be tolerated

def get_known_errors_for_firmware(current_fw_version):
    """Get list of known errors based on firmware version"""
    known_errors = []
    
    # Motion Module failure is a known issue starting before firmware 5.17.0.12
    if (rsutils.version(current_fw_version) < rsutils.version("5.17.0.12")):
        known_errors.append("Motion Module failure")  # See also RSDSO-20645
        # Add other version-specific known errors here as needed
    
    return known_errors

class FirmwareErrorMonitor:
    def __init__(self, known_errors=None):
        self.firmware_errors = []
        self.hardware_errors = []
        self.known_errors = []
        self.known_error_list = known_errors or []
        self.lock = threading.Lock()
    
    def notification_callback(self, notification):
        """Callback to handle firmware notifications"""
        category = notification.get_category()
        severity = notification.get_severity()
        description = notification.get_description()
        timestamp = notification.get_timestamp()
        
        with self.lock:
            log.d(f"Notification received - Category: {category}, Severity: {severity}, Description: {description}")
            
            # Check if this is a known error that should be ignored
            is_known_error = any(known_error in description for known_error in self.known_error_list)
            
            if is_known_error:
                error_info = {
                    'category': category,
                    'severity': severity,
                    'description': description,
                    'timestamp': timestamp
                }
                self.known_errors.append(error_info)
                log.w(f"Known error detected (ignored): {description}")
                return  # Don't count as failure
            
            # Check for hardware errors (firmware errors typically fall under this category)
            if category == rs.notification_category.hardware_error:
                error_info = {
                    'category': category,
                    'severity': severity, 
                    'description': description,
                    'timestamp': timestamp
                }
                self.hardware_errors.append(error_info)
                log.w(f"Hardware error detected: {description}")
            
            # Check for unknown errors which might include firmware issues
            elif category == rs.notification_category.unknown_error:
                error_info = {
                    'category': category,
                    'severity': severity,
                    'description': description, 
                    'timestamp': timestamp
                }
                self.firmware_errors.append(error_info)
                log.w(f"Unknown error detected: {description}")
    
    def get_error_count(self):
        """Get total number of firmware-related errors (excluding known errors)"""
        with self.lock:
            return len(self.firmware_errors) + len(self.hardware_errors)
    
    def get_error_summary(self):
        """Get summary of all detected errors"""
        with self.lock:
            return {
                'firmware_errors': self.firmware_errors.copy(),
                'hardware_errors': self.hardware_errors.copy(),
                'known_errors': self.known_errors.copy(),
                'total_errors': len(self.firmware_errors) + len(self.hardware_errors)
            }

with test.closure("Monitor firmware errors during streaming"):
    device, ctx = test.find_first_device_or_exit()
    
    # Get device info for logging
    device_name = device.get_info(rs.camera_info.name)
    device_serial = device.get_info(rs.camera_info.serial_number)
    firmware_version = device.get_info(rs.camera_info.firmware_version)
    
    log.i(f"Testing device: {device_name} (S/N: {device_serial}, FW: {firmware_version})")
    
    # Get known errors for this firmware version
    known_errors = get_known_errors_for_firmware(firmware_version)
    if known_errors:
        log.i(f"Known errors for firmware {firmware_version}: {', '.join(known_errors)}")
    else:
        log.i(f"No known errors defined for firmware {firmware_version}")
    
    # Set up error monitor with firmware-specific known errors
    error_monitor = FirmwareErrorMonitor(known_errors)
        
    # Query all available sensors and register notification callbacks
    sensors = device.query_sensors()
    registered_sensors = []
    
    for sensor in sensors:
        try:
            sensor_name = sensor.get_info(rs.camera_info.name)
            sensor.set_notifications_callback(error_monitor.notification_callback)
            registered_sensors.append(sensor_name)
            log.d(f"Registered notification callback for {sensor_name}")
        except Exception as e:
            sensor_name = "Unknown sensor"
            try:
                sensor_name = sensor.get_info(rs.camera_info.name)
            except Exception:
                pass
            log.w(f"Could not register notification callback for {sensor_name}: {e}")
    
    log.i(f"Monitoring firmware errors on {len(registered_sensors)} sensors: {', '.join(registered_sensors)}")
    
    # Start streaming
    pipe = rs.pipeline(ctx)
    profile = pipe.start()
    
    log.i(f"Started streaming, monitoring for firmware errors for {STREAMING_DURATION_SECONDS} seconds...")
    
    try:
        stopwatch = Stopwatch()
        frame_count = 0
        last_log_time = 0
        
        # Stream for specified duration while monitoring for errors
        while stopwatch.get_elapsed() < STREAMING_DURATION_SECONDS:
            try:
                frames = pipe.wait_for_frames()
                
                # Verify we're getting frames
                if frames:
                    frame_count += 1
                    elapsed = stopwatch.get_elapsed()
                    
                    # Log every second using stopwatch timing
                    if elapsed - last_log_time >= 1.0:
                        error_count = error_monitor.get_error_count()
                        log.d(f"Streaming progress: {elapsed:.1f}s, {frame_count} frames, {error_count} errors detected")
                        last_log_time = elapsed
                
            except RuntimeError as e:
                log.w(f"Frame timeout or error during streaming: {e}")
                # Continue monitoring even if individual frames fail
        
        elapsed_time = stopwatch.get_elapsed()
        log.i(f"Streaming completed - Duration: {elapsed_time:.1f}s, Total frames: {frame_count}")
        
    finally:
        pipe.stop()
        log.d("Pipeline stopped")
    
    # Analyze results
    error_summary = error_monitor.get_error_summary()
    total_errors = error_summary['total_errors']
    
    log.i(f"Firmware error monitoring results:")
    log.i(f"  - Total errors detected: {total_errors}")
    log.i(f"  - Hardware errors: {len(error_summary['hardware_errors'])}")
    log.i(f"  - Unknown/Firmware errors: {len(error_summary['firmware_errors'])}")
    log.i(f"  - Known errors (ignored): {len(error_summary['known_errors'])}")
    
    # Log detailed error information if any errors were found
    if total_errors > 0:
        log.w("Detailed error information:")
        
        for i, error in enumerate(error_summary['hardware_errors']):
            log.w(f"  Hardware Error {i+1}: {error['description']} (Severity: {error['severity']})")
        
        for i, error in enumerate(error_summary['firmware_errors']):
            log.w(f"  Firmware Error {i+1}: {error['description']} (Severity: {error['severity']})")
    
    # Log known errors for informational purposes
    if len(error_summary['known_errors']) > 0:
        log.i("Known errors detected (ignored for test result):")
        for i, error in enumerate(error_summary['known_errors']):
            log.i(f"  Known Error {i+1}: {error['description']} (Severity: {error['severity']})")
    
    # Test assertion - no firmware errors should be detected
    test.check_equal(total_errors, ERROR_TOLERANCE)
    
    log.i("Firmware error monitoring test completed successfully")

test.print_results_and_exit()
