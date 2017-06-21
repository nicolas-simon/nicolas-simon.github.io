////27/08/2015 21.21/////
(function(ext) {
   $.getScript('http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js');
   $.getScript('http://cdn.robotwebtools.org/roslibjs/current/roslib.min.js');
   //$.getScript('http://cdn.robotwebtools.org/EaselJS/current/easeljs.min.js');
   //$.getScript('http://cdn.robotwebtools.org/ros2djs/current/ros2d.min.js');

    //Ros Connection Vars
    var ros;
    var TestRosConnection=false;
    //Angles Position Vars
    var angle0=0;
    var angle1=0;
    var angle2=0;
    var angle3=0;
    var angle4=0;
    var angle5=0;
    var angle6=0;
    
    // Cleanup function when the extension is unloaded
    ext._shutdown = function() {};
    
    // Status reporting code
    // Use this to report missing hardware, plugin or unsupported browser
    ext._getStatus = function() {return {status: 2, msg: 'Ready'};};
    
    //Starting connection to websocket server
    //must first launch: "roslaunch rosbridge_server rosbridge_websocket.launch"
    ext.RosConnection = function(adress,port){
        try {
        ros = new ROSLIB.Ros({
        url : 'ws://'+adress+':'+port 
        });
        
        console.log('Loading '+'ws://'+adress+':'+port);
        } catch (err) {console.log('Unable to connecte to websocket')};
        
        ros.on('connection', function() {
        console.log('Connected to websocket server.');
        TestRosConnection=true;  
        });

        ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
        TestRosConnection=false;
        });

        ros.on('close', function() {
        console.log('Connection to websocket server closed.');
        TestRosConnection=false;
        });
    };
   //Test websocket connection
    ext.TestConnection = function() {
        try{
        ros.on('connection', function() {
        console.log('Connected to websocket server.');
        TestRosConnection=true;  
        });

        ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
        TestRosConnection=false;
        });

        ros.on('close', function() {
        console.log('Connection to websocket server closed.');
        TestRosConnection=false;
        });
   
        }catch(err){}
       return TestRosConnection;
       
    };
    
    //Moving the robot: publishing to cmd_vel
    ext.MoveRobot = function(direction,speed) {
       try{
        var cmdVel = new ROSLIB.Topic({
        ros : ros,
        name : '/turtle1/cmd_vel',
        messageType : 'geometry_msgs/Twist'
        });
        var twist = new ROSLIB.Message({
        linear : {x : 0,y : 0,z : 0},
        angular : {x : 0,y : 0,z : 0}
        });
        
if (direction == "Forward"){
   twist.linear.x=speed;
   console.log('Forward');
   
} 
else if (direction == "Backward"){
    twist.linear.x=-speed;
    console.log('Backward');
}  
else if (direction == "Right"){
    twist.angular.z=-speed;
    console.log('Right');
}  
else if (direction == "Left"){
    twist.angular.z=speed;
    console.log('Left');
}  
cmdVel.publish(twist);
console.log("Publishing cmd_vel");
    }catch(err) {console.log("Unable to Run MoveRobot Block")}; 
};
//Stopping the robot: publishing 0 to cmd_vel
ext.StopRobot = function(direction,speed) {
       try{
        var cmdVel = new ROSLIB.Topic({
        ros : ros,
        name : '/turtle1/cmd_vel',
        messageType : 'geometry_msgs/Twist'
        });
        var twist = new ROSLIB.Message({
        linear : {x : 0,y : 0,z : 0},
        angular : {x : 0,y : 0,z : 0}
        });
        cmdVel.publish(twist);
        console.log("Publishing cmd_vel");
        }catch(err) {console.log("Unable to Run StopRobot Block")}; 
};
      
    // Block and block menu descriptions
    var descriptor = {
        blocks: [
            // Block type, block name, function name
            ["h", "When connected to PC","TestConnection"],
            ["", "Connect to PC Adress: %s Port: %n","RosConnection","localhost","9090"],
            ["", "Move Turtle Direction %m.direction_menu Speed %n", "MoveRobot","Forward",0.2],
            ["", "Stop Turtle", "StopRobot"],
                        
        ],
        menus: {
            "direction_menu":["Forward","Backward","Right","Left"]
        },
    };

    // Register the extension
    ScratchExtensions.register('GR Turtlesim', descriptor, ext);
})({});
