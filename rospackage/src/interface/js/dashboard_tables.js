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

var vitesse_odom_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/wheel/odometry',
    messageType : 'nav_msgs/Odometry'
}); 

var debug_mot_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/debug_mot',
    messageType : 'std_msgs/Float32MultiArray'
}); 

var sonar_pairs_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/sonar_pairs',
    messageType : 'std_msgs/Float32MultiArray'
});

var estop_state_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/estop_state',
    messageType : 'std_msgs/Int32'
});

var soufflante_height_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/soufflante_height',
    messageType : 'std_msgs/Int32'
});

var gps_data_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/gps/fix',
    messageType : 'sensor_msgs/NavSatFix'
});

var imu_data_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/imu/data',
    messageType : 'sensor_msgs/Imu'
});

//---------------------------------------------------------
// ROS Subscribers
//---------------------------------------------------------
    
// We fill the HTML tables at the same time...
vitesse_odom_topic.subscribe(function(message) {
    console.log('Received message on ' + vitesse_odom_topic.name + ': ' + message.data);
    pos_table = document.getElementById('vitesse');
    pos_table.rows[1].cells[0].innerHTML = message.twist.twist.linear.x.toFixed(2);
    pos_table.rows[1].cells[1].innerHTML = message.twist.twist.linear.z.toFixed(2);
});

sonar_pairs_topic.subscribe(function(message) {
    console.log('Received message on ' + sonar_pairs_topic.name + ': ' + message.data);
    var i = message.data[0].toFixed(2);
    
    sonars_table = document.getElementById('Sonars');
    for (i = 0; i < 2; i++)
    {
        sonars_table.rows[message.data[0]+1+i].cells[1].innerHTML = message.data[i+1] < 2.0;
        sonars_table.rows[message.data[0]+1+i].cells[2].innerHTML = message.data[i+1].toFixed(2);
    }    
});

estop_state_topic.subscribe(function(message) {
    console.log('Received message on ' + estop_state_topic.name + ': ' + message.data);
    
    alarm_message = document.getElementById('Alarms');
    
    if (message.data > 0)
    {
        alarm_message.innerHTML+= '\n> ' + date_time + ': EStop State : ' + message.data[0].toFixed(0);
    }
});

debug_mot_topic.subscribe(function(message) {
    console.log('Received message on ' + debug_mot_topic.name + ': ' + message.data);
    
    motors_table = document.getElementById('Motors');
    motors_table.rows[2].cells[1].innerHTML = message.data[0].toFixed(2);
    motors_table.rows[2].cells[2].innerHTML = message.data[1].toFixed(2);
    motors_table.rows[2].cells[3].innerHTML = message.data[2].toFixed(2);
    motors_table.rows[2].cells[4].innerHTML = message.data[3].toFixed(2);

    motors_table.rows[3].cells[1].innerHTML = message.data[4].toFixed(2);
    motors_table.rows[3].cells[2].innerHTML = message.data[5].toFixed(2);
    motors_table.rows[3].cells[3].innerHTML = message.data[6].toFixed(2);
    motors_table.rows[3].cells[4].innerHTML = message.data[7].toFixed(2);
});

gps_data_topic.subscribe(function(message) {
    console.log('Received message on ' + gps_data_topic.name + ': ' + message.data);
    gps_table = document.getElementById('GPS');
    gps_table.rows[1].cells[0].innerHTML = message.longitude.toFixed(2);
    gps_table.rows[1].cells[1].innerHTML = message.latitude.toFixed(2);
    gps_table.rows[1].cells[2].innerHTML = message.altitude.toFixed(2);
});

imu_data_topic.subscribe(function(message) {
    console.log('Received message on ' + imu_data_topic.name + ': ' + message.data);
    
    imu_table = document.getElementById('IMU');
    imu_table.rows[1].cells[0].innerHTML = message.orientation.x.toFixed(2);
    imu_table.rows[1].cells[1].innerHTML = message.orientation.y.toFixed(2);
    imu_table.rows[1].cells[2].innerHTML = message.orientation.z.toFixed(2);
    imu_table.rows[1].cells[3].innerHTML = message.orientation.w.toFixed(2);
});

// TODO : Souffl Height

// Other alarms

// Camera
