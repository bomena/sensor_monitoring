import React, { useState, useEffect, useRef } from 'react';
import ROSLIB from 'roslib';

const SensorSyncStatus = () => {
  const ros = useRef(null);
  const [syncStatus, setSyncStatus] = useState({ is_synced: false, max_time_diff: 0 });

  useEffect(() => {
    if (!ros.current) {
      ros.current = new ROSLIB.Ros({ url: 'ws://localhost:9090' });

      ros.current.on('error', (error) => {
        console.log('Error connecting to ROS:', error);
      });

      ros.current.on('connection', () => {
        console.log('Connected to ROS.');
      });

      ros.current.on('close', () => {
        console.log('Connection to ROS closed.');
      });
    }

    const syncStatusListener = new ROSLIB.Topic({
      ros: ros.current,
      name: '/sensor_sync_status',
      messageType: 'std_msgs/String'
    });

    const handleUpdate = (message) => {
      try {
        const data = JSON.parse(message.data);
        setSyncStatus(data);
      } catch (error) {
        console.error('Error parsing JSON:', error);
      }
    };

    syncStatusListener.subscribe(handleUpdate);

    return () => {
      if (ros.current && ros.current.isConnected) {
        syncStatusListener.unsubscribe();
      }
    };
  }, []);

  const getButtonStyle = (isActive) => ({
    marginRight: '15px',
    backgroundColor: isActive ? 'blue' : 'red',
  });

  return (
    <div style={getButtonStyle(syncStatus.is_synced)}>
      <div style={{ padding: '10px' }}>
        <h1>Sensor Synchronization ( per 5s )</h1>
        <p style={{ fontSize: '25px' }}>Is Synced: {syncStatus.is_synced ? 'Yes' : 'No'}</p>
        <p style={{ fontSize: '25px' }}>Max Time Difference: {syncStatus.max_time_diff.toFixed(3)} s</p>
      </div>
    </div>
  );
};

export default SensorSyncStatus;
