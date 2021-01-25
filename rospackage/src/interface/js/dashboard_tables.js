/*
 * This script was created by Deneigus.
 * It is used to fill the HTML tables with ROS data.
 * Date: 2021-01-18
 * Rev: 0.00
 */

//---------------------------------------------------------
// Get date and time
//---------------------------------------------------------
var current = new Date();
console.log(current)
var date_time = current.toLocaleTimeString();

//---------------------------------------------------------
// Connection to ROS
//---------------------------------------------------------

// We create a ROS node to communicate with a rosbridge v2.0 server
// (we connect to localhost on the default port of 9090)
var ros = new ROSLIB.Ros({
url : 'ws://localhost:9090'
});

// Messages displayed in the web Consol and in the alarms section of the dashboard
ros.on('connection', function() {
    console.log('Connected to websocket server.');
    AlarmMessage = document.getElementById('Alarms');
    AlarmMessage.innerHTML+= '\n\n> ' + date_time + ': You are Connected to the ROS websocket server (localhost:9090).';
});

ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
    AlarmMessage = document.getElementById('Alarms');
    AlarmMessage.innerHTML+= '\n> ' + date_time + ': Error connecting to ROS websocket server (make sure Rosbridge is launched)';
});

ros.on('close', function() {
    console.log('Connection to websocket server closed.');
    AlarmMessage = document.getElementById('Alarms');
    AlarmMessage.innerHTML+= '\n> ' + date_time + ': Connection to ROS websocket server has been lost.';
});



//---------------------------------------------------------
// ROS Topics
//---------------------------------------------------------

var pos_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/pos',
    messageType : 'std_msgs/Float32MultiArray'
}); 

var obs_pos_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/obs_pos',
    messageType : 'std_msgs/Float32MultiArray'
});

var estop_state_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/estop_state',
    messageType : 'std_msgs/Float32MultiArray'
});

var tele_batt_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/tele_batt',
    messageType : 'std_msgs/Float32MultiArray'
});

var pos_tourellee_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/pos_tourelle',
    messageType : 'std_msgs/Float32MultiArray'
});

var debug_mot_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/debug_mot',
    messageType : 'std_msgs/Float32MultiArray'
});

var gps_data_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/gps_data',
    messageType : 'std_msgs/Float32MultiArray'
});

var imu_data_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/imu_data',
    messageType : 'std_msgs/Float32MultiArray'
});

//---------------------------------------------------------
// ROS Subscribers
//---------------------------------------------------------
    
// We fill the HTML tables at the same time...
pos_topic.subscribe(function(message) {
    console.log('Received message on ' + pos_topic.name + ': ' + message.data);
    var i;
    
    pos_table = document.getElementById('Position');
    for (i = 0; i < 3; i++)
    {
    	pos_table.rows[1].cells[i].innerHTML = message.data[i].toFixed(2);
    }    
});

obs_pos_topic.subscribe(function(message) {
    console.log('Received message on ' + obs_pos_topic.name + ': ' + message.data);
    var i;
    var j;
    
    sonars_table = document.getElementById('Sonars');
    for (i = 0; i < 8; i++)
    {
    	for (j = 0; j < 2; j++)
    	{
	    sonars_table.rows[i+1].cells[j+1].innerHTML = message.data[i*2+j].toFixed(2);
	}
    }    
});

estop_state_topic.subscribe(function(message) {
    console.log('Received message on ' + estop_state_topic.name + ': ' + message.data);
    
    alarm_message = document.getElementById('Alarms');
    
    if (message.data[0] > 0)
    {
        alarm_message.innerHTML+= '\n> ' + date_time + ': EStop State : ' + message.data[0].toFixed(0);
    }
});

tele_batt_topic.subscribe(function(message) {
    console.log('Received message on ' + tele_batt_topic.name + ': ' + message.data);
    var i;
    var j;
    
    batt_table = document.getElementById('Batteries');
    for (i = 0; i < 2; i++)
    {
    	for (j = 0; j < 4; j++)
    	{
	    batt_table.rows[i+1].cells[j+1].innerHTML = message.data[i*4+j].toFixed(2);
	}
    }      
});

pos_tourellee_topic.subscribe(function(message) {
    console.log('Received message on ' + pos_tourellee_topic.name + ': ' + message.data);    
});

debug_mot_topic.subscribe(function(message) {
    console.log('Received message on ' + debug_mot_topic.name + ': ' + message.data);
    var i;
    var j;
    
    motors_table = document.getElementById('Motors');
    for (i = 0; i < 5; i++)
    {
    	for (j = 0; j < 5; j++)
    	{
	    motors_table.rows[i+1].cells[j+1].innerHTML = message.data[i*5+j].toFixed(2);
	}
    }      
});

gps_data_topic.subscribe(function(message) {
    console.log('Received message on ' + gps_data_topic.name + ': ' + message.data);
    var i;
    
    gps_table = document.getElementById('GPS');
    for (i = 0; i < 3; i++)
    {
	gps_table.rows[1].cells[i+3].innerHTML = message.data[i].toFixed(2);
    }      
});

imu_data_topic.subscribe(function(message) {
    console.log('Received message on ' + imu_data_topic.name + ': ' + message.data);
    var i;
    
    imu_table = document.getElementById('IMU');
    for (i = 0; i < 9; i++)
    {
	imu_table.rows[1].cells[i].innerHTML = message.data[i].toFixed(2);
    }      
});



/*pos_topic.subscribe(function(message) {
    console.log('Received message on ' + general_topic.name + ': ' + message.data);

    gen_stats_table = document.getElementById('Gen_Stats');
    gen_stats_table.rows[0].cells[1].innerHTML = message.data;
    gen_stats_table.rows[1].cells[1].innerHTML = message.data;
    gen_stats_table.rows[2].cells[1].innerHTML = message.data;

    gps_table = document.getElementById('GPS');
    gps_table.rows[1].cells[0].innerHTML = message.data;
    gps_table.rows[1].cells[1].innerHTML = message.data;
    gps_table.rows[1].cells[2].innerHTML = message.data;
    gps_table.rows[1].cells[3].innerHTML = message.data;
    gps_table.rows[1].cells[4].innerHTML = message.data;
    gps_table.rows[1].cells[5].innerHTML = message.data;

    
    imu_table = document.getElementById('IMU');
    imu_table.rows[1].cells[0].innerHTML = message.data;
    imu_table.rows[1].cells[1].innerHTML = message.data;
    imu_table.rows[1].cells[2].innerHTML = message.data;
    imu_table.rows[1].cells[3].innerHTML = message.data;
    imu_table.rows[1].cells[4].innerHTML = message.data;
    imu_table.rows[1].cells[5].innerHTML = message.data;
    imu_table.rows[1].cells[6].innerHTML = message.data;
    imu_table.rows[1].cells[7].innerHTML = message.data;
    imu_table.rows[1].cells[8].innerHTML = message.data;
}); 
*/

// Other alarms

// Camera
