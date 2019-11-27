//====================================

var OldCounter = 0;
// ===================================
//
var light;
var canvas = document.getElementById("Light");
var context = canvas.getContext("2d");
context.font = '30pt Calibri';
context.arc(20, 20, 20, 0, Math.PI * 2, false);
//====================================

var OrangeBox = [];
var GreenBox = [];
var BlueBox = [];
var YellowBox = [];
var WhiteBox = [];

//=======================================
var ButtonFlag = 0;

//=======================================
var video_canvas = document.getElementById("playerfunction");
//=======================================
var x=0;
var y=0;
var w=0;
var sigma=0;

var reset_bool=false;
var reset_x=0;
var reset_y=0;
var reset_w=0;

var points = [[100,95], [363,95], [577,95], [840,95],
              [175,195],[304,225],[471,203],[639,224],
              [766,195],[133,332],[263,333],[678,332],
              [809,332],[175,469],[303,442],[472,463],
              [640,442],[768,470],[104,571],[365,571],
              [579,571],[840,571]];

$("input:radio").attr("checked", false);

document.getElementById("KeyboardControlButton").checked = true;
document.getElementById("StandbyButton").checked = true;
document.getElementById("coord1input").checked = true;
