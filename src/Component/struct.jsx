import React from 'react';
import MyMap from './MyMap';
// import RosbagControl from './Record';
import RosbagStatus from './RecordCheck';
import DataStreamButtons from './SensorState';
import SensorSyncStatus from './Sync';
import RosbagSize from './FileSize';
import Sensor from './Sensor';
import '../index.css'


const SetViz = (props) => {

  return (
    <>
      <div className="grid">
        <div>
          <RosbagSize />
          <h2>Recording Status</h2>
          <RosbagStatus />
        </div>
        <div className='sensor_sync'>
          <SensorSyncStatus />
        </div>
        <div className='mapOnly'>
          <MyMap />
        </div>
        <div className='sensor_title'>
          <h2>Sensor Connectivity Status</h2>
        </div>
        <div className='sensor_state'>
          <DataStreamButtons />
        </div>
        <div className='sensor_display'>
          <Sensor />
        </div>
        <div>
          
        </div>
      </div>
    </>   

  )
}

export default SetViz;