// Connecting to ROS
// -----------------
var ros = new ROSLIB.Ros();

// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function(error) {
  document.getElementById('connecting').style.display = 'none';
  document.getElementById('connected').style.display = 'none';
  document.getElementById('closed').style.display = 'none';
  document.getElementById('error').style.display = 'inline';
  console.log(error);
});

ros.on('error', function(error) {
  console.log('Maybe on k3s cluster, attempting to connect to localhost:30001')
  ros.connect('ws://localhost:30001')
})

// Find out exactly when we made a connection.
ros.on('connection', function() {
  console.log('Connection made!');
  document.getElementById('connecting').style.display = 'none';
  document.getElementById('error').style.display = 'none';
  document.getElementById('closed').style.display = 'none';
  document.getElementById('connected').style.display = 'inline';
});

ros.on('close', function() {
  console.log('Connection closed.');
  document.getElementById('connecting').style.display = 'none';
  document.getElementById('connected').style.display = 'none';
  document.getElementById('closed').style.display = 'inline';
});

// Create a connection to the rosbridge WebSocket server.
ros.connect('ws://'+window.location.hostname+':9090');

// Publish a Topic
var example = new ROSLIB.Topic({
  ros : ros,
  name : '/emergency_stop',
  messageType : 'std_msgs/String'
});

// Publish a Topic
var start_publisher = new ROSLIB.Topic({
  ros : ros,
  name : '/mission_start',
  messageType : 'std_msgs/String'
});

function sendEmergencyStop() {
  var message = 'EMERGENCY STOP';
  example.publish({data: message});
  console.log("Emergency Stop Pressed")
  $('#eStopSentMessage').show()
  document.getElementById('eStopButton').style.filter="brightness(50%)"
  setTimeout(function() {
      $('#eStopSentMessage').hide()
      document.getElementById('eStopButton').style.filter="brightness(100%)"
    }, 1000);
}

function sendMissionStart() {
  var message = 'MISSION START';
  start_publisher.publish({data: message})
  console.log("Mission Start Pressed")
  $('#missionStartSentMessage').show()
  document.getElementById('missionStartButton').style.filter="brightness(50%)"
  setTimeout(function() {
      $('#missionStartSentMessage').hide()
      document.getElementById('missionStartButton').style.filter="brightness(100%)"
    }, 1000);
}

document.getElementById('eStopButton').addEventListener('click',sendEmergencyStop);
document.getElementById('missionStartButton').addEventListener('click',sendMissionStart);