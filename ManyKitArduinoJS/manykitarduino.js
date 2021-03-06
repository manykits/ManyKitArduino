// manykitarduino.js

ManyKitSerial = require("./manykitserial")
ManyKitUDP = require("./manykitudp")

// "100", //OT_TOGET_NETID
// "101", //OT_RETRUN_NETID
// "0",  //OT_PM
// "1",  //OT_DW
// "2",  //OT_AW
// "3",  //OT_RETURN_DR
// "4",  //OT_RETURN_AR
// "5",  //OT_SVR_I
// "6",  //OT_SVR_W
// "7",  //OT_DST_I
// "8",  //OT_DST_T
// "9",  //OT_RETURN_DIST
// "10", //OT_MOTO_I
// "11", //OT_MOTO_RUN
// "12", //OT_MOTO_RUNSIMPLE
// "13", //OT_MOTO_STOP
// "14", //OT_MOTO_I_SPD
// "15", //OT_RETURN_MOTOSPD
// "16", //OT_MOTO_I_DRIVER4567
// "17", //OT_MOTO_I_DRIVER298N
// "18", //OT_MP3_INIT
// "19", //OT_MP3_DO
// "20", //OT_MP3_PLAYFOLDER
// "21", //OT_MP3_SETVOLUME
// "24", //OT_IR_INIT
// "25", //OT_IR_SEND
// "26", //OT_RETURN_IR
// "27", //OT_HX711_I
// "28", //OT_HX711_TEST
// "29", //OT_RETURN_HX711
// "30", //OT_DSTMAT_I,
// "31", //OT_RETURN_DSTMAT,
// "32", //OT_AXIS_I,
// "33", //OT_RETURN_AXIS
// "34", //OT_SET_TIME
// "35", //OT_RC_INIT
// "36", //OT_RC_SEND
// "37", //OT_RETRUN_RC
// "38", //OT_DHT_I
// "39", //OT_RETURN_DHTTEMP
// "40", //OT_RETURN_DHTHUMI
// "41", //OT_LEDSTRIP_I
// "42", //OT_LEDSTRIP_SET
// "43", //OT_SEGMENT_INIT
// "44", //OT_SEGMENT_BRIGHTNESS
// "45", //OT_SEGMENT_CLEAR
// "46", //OT_SEGMENT_DISPLAY
// "47", //OT_LEDMATRIX_I
// "48", //OT_LEDMATRIX_BRIGHTNESS
// "49", //OT_LEDMATRIX_CLEARSCREEN
// "50", //OT_LEDMATRIX_LIGHTAT
// "51", //OT_STEPMOTO_I
// "52", //OT_STEPMOTO_ENABLE
// "53", //OT_STEPMOTO_DIR
// "54", //OT_STEPMOTO_STEP
// "150", //OT_MC_INTERNAL_LIGHT
// "151", //OT_MC_LIGHT
// "152", //OT_MC_SEGMENT
// "153", //OT_MC_MOTO
// "154", //OT_MC_DISTTEST
// "161", //OT_MB_MOTO
// "162", //OT_MB_SEND
// "163", //OT_MB_BUZZER
// "200" //OT_VERSION

function convertToArduino(cntStr) {
    var str = "";
    var cmdStrs = cntStr.split(',');
    var cmd0 = "";
    var cmd1 = "";
    var cmd2 = "";
    var cmd3 = "";
    var cmd4 = "";
    var cmd5 = "";
    var cmd6 = "";
    var cmd7 = "";
    var cmd8 = "";
    if (cmds.length > 0)
        cmd0 = cmds[0];
    if (cmds.length > 1)
        cmd1 = cmds[1];
    if (cmds.length > 2)
        cmd2 = cmds[2];
    if (cmds.length > 3)
        cmd3 = cmds[3];
    if (cmds.length > 4)
        cmd4 = cmds[4];
    if (cmds.length > 5)
        cmd5 = cmds[5];
    if (cmds.length > 6)
        cmd6 = cmds[6];
    if (cmds.length > 7)
        cmd7 = cmds[7];
    if (cmds.length > 8)
        cmd8 = cmds[8];

    return str;
}

function pinToArduino(str) {
    if ("0" == str)
        return "0";
    else if ("1" == str)
        return "1";
    else if ("2" == str)
        return "2";
    else if ("3" == str)
        return "3";
    else if ("4" == str)
        return "4";
    else if ("5" == str)
        return "5";
    else if ("6" == str)
        return "6";
    else if ("7" == str)
        return "7";
    else if ("8" == str)
        return "8";
    else if ("9" == str)
        return "9";
    else if ("10" == str)
        return "10";
    else if ("11" == str)
        return "11";
    else if ("12" == str)
        return "12";
    else if ("13" == str)
        return "13";
    else if ("A0" == str)
        return "30";
    else if ("A1" == str)
        return "31";
    else if ("A2" == str)
        return "32";
    else if ("A3" == str)
        return "33";
    else if ("A4" == str)
        return "34";
    else if ("A5" == str)
        return "35";
    else if ("A6" == str)
        return "36";

    return "0";
}

function pinToIndex(str) {
    if ("0" == str)
        return 0;
    else if ("1" == str)
        return 1;
    else if ("2" == str)
        return 2;
    else if ("3" == str)
        return 3;
    else if ("4" == str)
        return 4;
    else if ("5" == str)
        return 5;
    else if ("6" == str)
        return 6;
    else if ("7" == str)
        return 7;
    else if ("8" == str)
        return 8;
    else if ("9" == str)
        return 9;
    else if ("10" == str)
        return 10;
    else if ("11" == str)
        return 11;
    else if ("12" == str)
        return 12;
    else if ("13" == str)
        return 13;
    else if ("A0" == str)
        return 14;
    else if ("A1" == str)
        return 15;
    else if ("A2" == str)
        return 16;
    else if ("A3" == str)
        return 17;
    else if ("A4" == str)
        return 18;
    else if ("A5" == str)
        return 19;
    else if ("A6" == str)
        return 20;

    return 0;
}

function pinModeToArduino(str) {
    if ("INPUT" == str || "input" == str)
        return "0";

    return "1";
}

function highLowToArduino(str) {
    if ("HIGH" == str || "high" == str)
        return "1";
    else if ("LOW" == str || "low" == str)
        return "0";

    return str;
}

function dirToArduino(str) {
    if ("none" == str) {
        return 0;
    }
    else if ("forward" == str) {
        return 1;
    }
    else if ("backward" == str) {
        return 2;
    }

    return 0;
}

function dirToArduinoSimple(str) {
    if ("none" == str) {
        return 0;
    }
    else if ("go" == str) {
        return 1;
    }
    else if ("back" == str) {
        return 2;
    }
    else if ("left" == str) {
        return 3;
    }
    else if ("right" == str) {
        return 4;
    }

    return 0;
}

function mp3DoTypeToArduino(strParam1) {
    // enum MP3PlayType
    // {
    // 	MPT_PLAY = 0,
    // 	MPT_PAUSE,
    // 	MPT_STOP,
    // 	MPT_NEXT,
    // 	MPT_BEFORE,
    // 	MPT_RANDOM,
    // 	MPT_LOOP_SINGLE,
    // 	MPT_LOOP_SINGLE_CLOSE,
    // 	MPT_LOOP_ALL,
    // 	MPT_LOOP_ALL_CLOSE,
    // 	MPT_VOLUME_INCREASE,
    // 	MPT_VOLUME_DECREASE,
    // 	MPT_MAX_TYPE
    // };

    if ("play" == strParam1) {
        type = 0;
    }
    else if ("pause" == strParam1) {
        type = 1;
    }
    else if ("stop" == strParam1) {
        type = 2;
    }
    else if ("next" == strParam1) {
        type = 3;
    }
    else if ("before" == strParam1) {
        type = 4;
    }
    else if ("random" == strParam1) {
        type = 5;
    }
    else if ("loop_single" == strParam1) {
        type = 6;
    }
    else if ("loop_single_close" == strParam1) {
        type = 7;
    }
    else if ("loop_all" == strParam1) {
        type = 8;
    }
    else if ("loop_all_close" == strParam1) {
        type = 9;
    }
    else if ("volume_increase" == strParam1) {
        type = 10;
    }
    else if ("volume_decrease" == strParam1) {
        type = 11;
    }

    return type;
}

function mp3FolderToArduino(param0) {
    if ("01" == param0)
        return "1";
    else if ("02" == param0)
        return "2";
    else if ("03" == param0)
        return "3";
    else if ("04" == param0)
        return "4";
    else if ("05" == param0)
        return "5";
    else if ("06" == param0)
        return "6";
    else if ("07" == param0)
        return "7";
    else if ("08" == param0)
        return "8";
    else if ("09" == param0)
        return "9";

    return "1";
}

function boolToArduino(param0) {
    if (true == param0) {
        return 1;
    }
    return 0;
}

ManyKitArduino = module.exports = function (url, port, options, callback) {

    ManyKitArduino.prototype.onReady = function () {
        var self = this;

        console.log("ManyKitArduino.prototype.ready");

        // manykit
        self.pins = [];
        for (var i = 0; i < 30; i++) {
            self.pins.push({
                value: 0,
                report: 1,
            });
        }
        self.dists = [];
        for (var i = 0; i < 8; i++) {
            self.dists.push({
                value: 0
            });
        }
        self.irRecvValue = 0;
        self.DHTTemperature = 0.0;
        self.DHTHumidity = 0.0;

        self.isReady = true;

        if (null != self.callback) {
            self.callback("ready", "");
        }
    };

    var self = this;

    self.callback = callback;

    var defaults = {
        sp: {
            baudRate: 9600,
            bufferSize: 256,
        },
    };
    var settings = Object.assign({}, defaults, options);

    self.isReady = false;

    self.SERIAL_MODES = {
        CONTINUOUS_READ: 0x00,
        STOP_READING: 0x01,
    };

    self.settings = settings;

    self.name = "ManyKitArduino";
    self.HIGH = 1;
    self.LOW = 0;
    self.pins = [];
    self.ports = Array(16).fill(0);
    self.versionReceived = false;
    self.pending = 0;

    // manykit
    self.dists = [];
    self.irRecvValue = 0;
    self.DHTTemperature = 0.0;
    self.DHTHumidity = 0.0;

    self.url = url;
    self.port = port;

    if ("" == url)
    {
        self.manykitSerial = new ManyKitSerial(self.port, settings.sp.baudRate);

        self.manykitSerial.setFunOnClose(function () {
            console.log("transport close");
            self.versionReceived = false;
        }.bind(self));

        self.manykitSerial.setFunOnDisconnect(function () {
            console.log("transport disconnect");
        }.bind(self));

        self.manykitSerial.setFunOnOpen(function () {
            console.log("transport open");
        }.bind(self));

        self.manykitSerial.setFunOnError(function (error) {
            console.log("transport error");
        }.bind(self));
    }
    else{
         self.manykitUDP = new ManyKitUDP(self.url, self.port);
    }

    function byteToString(arr) {
        if (typeof arr === 'string') {
            return arr;
        }
        var str = '',
            _arr = arr;
        for (var i = 0; i < _arr.length; i++) {
            var one = _arr[i].toString(2),
                v = one.match(/^1+?(?=0)/);
            if (v && one.length == 8) {
                var bytesLength = v[0].length;
                var store = _arr[i].toString(2).slice(7 - bytesLength);
                for (var st = 1; st < bytesLength; st++) {
                    store += _arr[st + i].toString(2).slice(2);
                }
                str += String.fromCharCode(parseInt(store, 2));
                i += bytesLength - 1;
            } else {
                str += String.fromCharCode(_arr[i]);
            }
        }
        return str;
    }

    function arduinoToPin(val) {
        return val;
    }

    self.recvAllStr = "";
    ManyKitArduino.prototype.onRecvData = function (data) {
        var self = this;

        for (var i = 0; i < data.length; i++) {
            var chara = String.fromCharCode(data[i]);

            if ('\r' == chara) {
                continue;
            }
            else if ('\n' == chara) {
                var str0 = byteToString(self.recvAllStr);
                var str = str0.substr(4);
                var cmds = str.split(" ");

                var cmdStr = "";
                var paramStr = "";
                var paramStr1 = "";
                var paramStr2 = "";
                var paramStr3 = "";
                var paramStr4 = "";

                if (cmds.length > 0)
                    cmdStr = cmds[0];

                if (cmds.length > 1)
                    paramStr = cmds[1];

                if (cmds.length > 2)
                    paramStr1 = cmds[2];

                if (cmds.length > 3)
                    paramStr3 = cmds[3];

                if (cmds.length > 4)
                    paramStr4 = cmds[4];

                if ("200" == cmdStr) {
                    if (!self.versionReceived) {
                        self.versionReceived = true;
                        console.log("versionReceived!!!!!!!!!!!!!!!");

                        self.onReady();
                    }
                }
                else if ("3" == cmdStr) { // OT_RETURN_DR
                    var pinIndex = arduinoToPin(paramStr);

                    var val = parseInt(paramStr1);
                    self.pins[pinIndex].value = val;
                }
                else if ("4" == cmdStr) { // OT_RETURN_AR
                    var pinIndex = arduinoToPin(paramStr);

                    var val = parseInt(paramStr1);
                    self.pins[pinIndex].value = val;
                }
                else if ("9" == cmdStr) { // OT_RETURN_DIST
                    var val = parseFloat(paramStr);
                    self.dists[0].value = val;
                }
                else if ("26" == cmdStr) { // OT_RETURN_IR
                    var val = parseInt(paramStr);
                    self.irRecvValue = val;
                }
                else if ("39" == cmdStr) {// OT_RETURN_DHTTEMP
                    self.DHTTemperature = parseFloat(paramStr);
                    self.DHTHumidity = parseFloat(paramStr1);
                }

                self.recvAllStr = "";
            }
            else {
                self.recvAllStr += chara;
            }
        }
    }
    if (null != self.manykitSerial)
        self.manykitSerial.setFunOnData(self.onRecvData.bind(self));
    if (null != self.manykitUDP)
        self.manykitUDP.setFunOnData(self.onRecvData.bind(self));

    ManyKitArduino.prototype.writeData = function (dataStr) {
        if (null != self.manykitSerial)
            self.manykitSerial.write(dataStr);

        if (null != self.manykitUDP)
            self.manykitUDP.writeUDP(self.url, self.port, dataStr);
    }

    ManyKitArduino.prototype.getDigitalPinValue = function (pin) {
        return self.pins[pin].value;
    };

    ManyKitArduino.prototype.getAnalogPinValue = function (aPin) {
        return self.pins[aPin].value;
    };

    ManyKitArduino.prototype.getUltrasonicDistance = function () {
        return self.dists[0].value;
    };

    ManyKitArduino.prototype.getIRRecvValue = function () {
        return self.irRecvValue;
    };

    ManyKitArduino.prototype.getDHTTempture = function () {
        return self.DHTTemperature;
    };

    ManyKitArduino.prototype.getDHTHumidity = function () {
        return self.DHTHumidity;
    };

    ManyKitArduino.prototype.isBoardReady = function () {
        return self.versionReceived;
    };

    // pin
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13',
    // 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    // mode
    // 'INPUT', 'OUTPUT'
    ManyKitArduino.prototype.pinMode = function (pin, mode) {
        var self = this;
        // OT_PM
        //var cntStr = "0" + "," + pin + "," + mode; 
        var cntStr = "0000" + "0" + " " + pinToArduino(pin) + " " + pinModeToArduino(mode) + "\n";
        self.writeData(cntStr);
    }

    // pin
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    // val
    // "HIGH","LOW" /"high","low"
    ManyKitArduino.prototype.digitalWrite = function (pin, val) {
        var self = this;
        // OT_DW
        //var cntStr = "1" + "," + pin + "," + val;   
        var cntStr = "0000" + "1" + " " + pinToArduino(pin) + " " + highLowToArduino(val) + "\n";
        self.writeData(cntStr);
    }

    // pin
    // '3', '5', '6', '9', '10', '11'
    // val
    ManyKitArduino.prototype.pwmWrite = function (pin, val) {
        var self = this;
        // OT_AW
        //var cntStr = "2" + "," + pin + "," + val;
        var cntStr = "0000" + "2" + " " + pinToArduino(pin) + " " + val + "\n";
        self.writeData(cntStr);
    },

    // pin
    // 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    // val
    ManyKitArduino.prototype.analogWrite = function (pin, val) {
        var self = this;
        // OT_AW
        //var cntStr = "2" + "," + pin + "," + val; 
        var cntStr = "0000" + "2" + " " + pinToArduino(pin) + " " + val + "\n";
        self.writeData(cntStr);
    }

    // pin
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    ManyKitArduino.prototype.digitalRead = function (pin) {
        var self = this;
        // "OT_RETURN_DR
        //var cntStr = "3" + "," + pin;
        var cntStr = "0000" + "3" + " " + pinToArduino(pin) + "\n";
        self.writeData(cntStr);

        var index = pinToIndex(pin);
        return self.getDigitalPinValue(index);
    }

    // pin
    // 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    ManyKitArduino.prototype.analogRead = function (pin) {
        var self = this;

        // OT_RETURN_AR
        //var cntStr = "4" + "," + pin;
        var cntStr = "0000" + "4" + " " + pinToArduino(pin) + "\n";
        self.writeData(cntStr);

        var index = pinToIndex(pin);
        return self.getAnalogPinValue(index);
    }

    // index
    // 0-5
    // pin
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    ManyKitArduino.prototype.servoInit = function (index, pin) {
        var self = this;

        // OT_SVR_I
        //var cntStr = "5" + "," + index + "," + pin;
        var cntStr = "0000" + "5" + " " + index + " " + pinToArduino(pin) + "\n";
        self.writeData(cntStr);
    }

    // index
    // 0-5
    ManyKitArduino.prototype.servoWrite = function (index, val) {
        var self = this;

        // OT_SVR_W
        //var cntStr = "6" + "," + index + "," + val;
        var cntStr = "0000" + "6" + " " + index + " " + val + "\n";
        self.writeData(cntStr);
    }

    // dist
    // pin
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    // pin1
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    ManyKitArduino.prototype.ultrasonicInit = function (pin, pin1) {
        var self = this;

        // OT_DST_I
        //var cntStr = "7" + "," + pin + "," + pin1;
        var cntStr = "0000" + "7" + " " + pinToArduino(pin) + " " + pinToArduino(pin1) + "\n";
        self.writeData(cntStr);
    }

    ManyKitArduino.prototype.ultrasonicTest = function () {
        var self = this;

        // OT_DST_T
        //var cntStr = "8";
        var cntStr = "0000" + "8" + "\n";
        self.writeData(cntStr);
    }

    ManyKitArduino.prototype.ultrasonicMeasure = function () {
        var self = this;

        // OT_RETURN_DIST
        //var cntStr = "9";
        var cntStr = "0000" + "9" + "\n";
        Arduino.getURL('arduino', sprite.arduino.board, cntStr, null);

        return self.getUltrasonicDistance();
    }

    ManyKitArduino.prototype.motoInit10111213 = function () {
        var self = this;

        // OT_MOTO_I
        //var cntStr = "10";
        var cntStr = "0000" + "10" + "\n";
        self.writeData(cntStr);
    }

    // index
    // 0-1
    // dir
    // 'none', 'forward', 'backward'
    // speed
    // 0-255
    ManyKitArduino.prototype.vehicleRun = function (index, dir, speed) {
        var self = this;

        // OT_MOTO_RUN
        //var cntStr = "11" + "," + index + "," + dir + "," + speed;
        var cntStr = "0000" + "11" + " " + index + " " + dirToArduino(dir) + " " + speed + "\n";
        self.writeData(cntStr);
    }

    // dir
    // 'none','go','back','left','right'
    // speed
    // 0-255
    ManyKitArduino.prototype.vehicleSimpleRun = function (dir, speed) {
        var self = this;

        // OT_MOTO_RUNSIMPLE
        //var cntStr = "12" + "," + dir + "," + speed;
        var cntStr = "0000" + "12" + " " + dirToArduinoSimple(dir) + " " + speed + "\n";
        self.writeData(cntStr);
    }

    ManyKitArduino.prototype.vehicleStop = function () {
        var self = this;

        // OT_MOTO_STOP
        //var cntStr = "13";
        var cntStr = "0000" + "13" + "\n";
        self.writeData(cntStr);
    }

    // pinLA
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    // pinLB
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    // pinRA
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    // pinRB
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    ManyKitArduino.prototype.vehicleSpeedEncorderInit = function (pinLA, pinLB, pinRA, pinRB) {
        var self = this;

        // OT_MOTO_I_SPD
        //var cntStr = "14" + "," + pinLA + "," + pinLB + "," + pinRA + "," + pinRB;
        var cntStr = "0000" + "14" + " " + pinToArduino(pinLA) + " " + pinToArduino(pinLB) + " " + pinToArduino(pinRA) + " " + pinToArduino(pinRB) + "\n";
        self.writeData(cntStr);
    }

    // index
    // 0-1
    ManyKitArduino.prototype.vehicleGetSpeed = function (index) {
        var self = this;

        // OT_RETURN_MOTOSPD
        //var cntStr = "15" + "," + index;
        var cntStr = "0000" + "15" + " " + index + "\n";
        self.writeData(cntStr);
        return 0.0;
    }

    ManyKitArduino.prototype.motoInit4567 = function () {
        var self = this;

        // OT_MOTO_I_DRIVER4567
        //var cntStr = "16";
        var cntStr = "0000" + "16" + "\n";
        self.writeData(cntStr);
    }

    // pinL0
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    // pinL1
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    // pinLS
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    // pinR0
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    // pinR1
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    // pinRS
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    ManyKitArduino.prototype.vehicleInit_MotoBoard298N = function (pinL0, pinL1, pinLS, pinR0, pinR1, pinRS) {
        var self = this;

        // OT_MOTO_I_DRIVER298N
        //var cntStr = "17" + "," + pinL0 + "," + pinL1 + "," + pinLS + "," + pinR0 + "," + pinR1 + "," + pinRS;
        var cntStr = "0000" + "17" + " " + pinToArduino(pinL0)
            + " " + pinToArduino(pinL1)
            + " " + pinToArduino(pinLS)
            + " " + pinToArduino(pinR0)
            + " " + pinToArduino(pinR1)
            + " " + pinToArduino(pinRS) + "\n";
        self.writeData(cntStr);
    }

    // pinCLK
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    // pinData
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    ManyKitArduino.prototype.segmentInit = function (pinCLK, pinData) {
        var self = this;

        // OT_SEGMENT_INIT
        //var cntStr = "43" + "," + pinCLK + "," + pinData;
        var cntStr = "0000" + "43" + " " + pinToArduino(pinCLK) + " " + pinToArduino(pinData) + "\n";
        self.writeData(cntStr);
    }

    ManyKitArduino.prototype.segmentSetBrightness = function (val) {
        var self = this;

        // OT_SEGMENT_BRIGHTNESS
        //var cntStr = "44" + "," + val;
        var cntStr = "0000" + "44" + " " + val + "\n";
        self.writeData(cntStr);
    }

    ManyKitArduino.prototype.segmentClear = function () {
        var self = this;

        // OT_SEGMENT_CLEAR
        //var cntStr = "45";
        var cntStr = "0000" + "45" + "\n";
        self.writeData(cntStr);
    }

    ManyKitArduino.prototype.segmentDisplayInt = function (val) {
        var self = this;

        // OT_SEGMENT_DISPLAY
        //var cntStr = "46" + "," + "1" + "," + val;
        var cntStr = "0000" + "46" + " " + 1 + " " + val + "\n";
        self.writeData(cntStr);
    }

    ManyKitArduino.prototype.segmentDisplayFloat = function (val) {
        var self = this;

        // OT_SEGMENT_DISPLAY
        //var cntStr = "46" + "," + "2" + "," + val;
        var cntStr = "0000" + "46" + " " + "2" + " " + val + "\n";
        self.writeData(cntStr);
    }

    // pinR
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    // pinT
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    ManyKitArduino.prototype.MP3Init = function (pinR, pinT) {
        var self = this;

        // OT_MP3_INIT
        //var cntStr = "18" + "," + pinR + "," + pinT;
        var cntStr = "0000" + "18" + " " + pinToArduino(pinT) + " " + pinToArduino(pinR) + "\n";
        self.writeData(cntStr);
    }

    // type
    // 'play','pause','stop','next','before','random','loop_single','loop_single_close','loop_all','loop_all_close','volume_increase','volume_decrease'
    ManyKitArduino.prototype.MP3DO = function (type) {
        var self = this;

        // OT_MP3_DO
        //var cntStr = "19" + "," + type;
        var cntStr = "0000" + "19" + " " + mp3DoTypeToArduino(type) + "\n";
        self.writeData(cntStr);
    }

    // param0
    // '01','02','03','04','05','06','07','08','09'
    // param1
    // 1 2 3 4 5 6 7 8...
    ManyKitArduino.prototype.MP3Play = function (param0, param1) {
        var self = this;

        // OT_MP3_PLAYFOLDER
        //var cntStr = "20" + "," + param0 + "," + param1;
        var cntStr = "0000" + "20" + " " + mp3FolderToArduino(param0) + " " + param1 + "\n";
        self.writeData(cntStr);
    }

    // param0
    // 0-30
    ManyKitArduino.prototype.MP3SetVolume = function (param0) {
        var self = this;

        // OT_MP3_SETVOLUME
        //var cntStr = "21" + "," + param0;
        var cntStr = "0000" + "21" + " " + param0 + "\n";
        self.writeData(cntStr);
    }

    // pinR
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    ManyKitArduino.prototype.IRInit = function (pinR) {
        var self = this;

        // OT_MP3_VOLUME
        //var cntStr = "24" + "," + pinR;
        var cntStr = "0000" + "24" + " " + pinToArduino(pinR) + "\n";
        self.writeData(cntStr);
    }

    ManyKitArduino.prototype.IRSendfunction = function (val) {
        var self = this;

        // OT_IR_INIT
        //var cntStr = "25" + "," + val;
        var cntStr = "0000" + "25" + " " + val + "\n";
        self.writeData(cntStr);
    }

    ManyKitArduino.prototype.IRReceivedValue = function () {
        var self = this;

        // OT_RETURN_IR
        //var cntStr = "26";
        var cntStr = "0000" + "26" + "\n";
        self.writeData(cntStr);

        return self.getIRRecvValue();
    }

    // pin
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    ManyKitArduino.prototype.DHTInit = function (pin) {
        var self = this;

        // OT_DHT_I
        //var cntStr = "38" + "," + pin;
        var cntStr = "0000" + "38" + " " + pinToArduino(pin) + "\n";
        self.writeData(cntStr);
    }

    ManyKitArduino.prototype.GetTemperature = function () {
        return self.getDHTTempture();
    }

    ManyKitArduino.prototype.GetHumidity = function () {
        return self.getDHTHumidity();
    }

    // rc init
    // pin
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    ManyKitArduino.prototype.RCInit = function (pin) {
        var self = this;

        // OT_RC_INIT
        //var cntStr = "35" + "," + pin;
        var cntStr = "0000" + "35" + " " + pinToArduino(pin) + "\n";
        self.writeData(cntStr);
    }

    ManyKitArduino.prototype.RCSend = function (val) {
        var self = this;

        // OT_RC_SEND
        //var cntStr = "36" + "," + val;
        var cntStr = "0000" + "36" + " " + val + "\n";
        self.writeData(cntStr);
    }

    // pin
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    ManyKitArduino.prototype.RGBLEDInit = function (pin, num) {
        var self = this;

        // OT_LEDSTRIP_I
        //var cntStr = "41" + "," + pin + "," + num;
        var cntStr = "0000" + "41" + " " + pinToArduino(pin) + " " + num + "\n";
        self.writeData(cntStr);
    }

    // index
    // 0....
    // r g b
    // 0-255
    ManyKitArduino.prototype.RGBLEDSetColor = function (index, r, g, b) {
        var self = this;

        // OT_LEDSTROP_SET
        //var cntStr = "42" + "," + index + "," + r + "," + g + "," + b;
        var cntStr = "0000" + "42" + " " + index + " " + r + " " + g + " " + b + "\n";
        self.writeData(cntStr);
    }

    // pinClk
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    // pinData
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    ManyKitArduino.prototype.LEDMatrixInit = function (pinClk, pinData) {
        var self = this;

        // OT_LEDMATRIX_I
        //var cntStr = "47" + "," + pinCLK + "," + pinData;
        var cntStr = "0000" + "47" + " " + pinToArduino(pinClk) + " " + pinToArduino(pinData) + "\n";
        self.writeData(cntStr);
    }

    // val
    ManyKitArduino.prototype.LEDMatrixSetBrightness = function (val) {
        var self = this;

        // OT_LEDMATRIX_BRIGHTNESS
        //var cntStr = "48" + "," + val;
        var cntStr = "0000" + "48" + " " + val + "\n";
        self.writeData(cntStr);
    }

    ManyKitArduino.prototype.LEDMatrixClearScreen = function () {
        var self = this;

        // OT_LEDMATRIX_CLEARSCREEN
        //var cntStr = "49";
        var cntStr = "0000" + "49" + "\n";
        self.writeData(cntStr);
    }

    // onoff
    // true false
    ManyKitArduino.prototype.LEDMatrixLightAt = function (x, y, width, onOff) {
        var self = this;

        // OT_LEDMATRIX_LIGHTAT
        //var cntStr = "50" + "," + x + "," + y + "," + width + "," + onOff;
        var cntStr = "0000" + "50" + " " + x + " " + y + " " + width + " " + boolToArduino(onOff) + "\n";
        self.writeData(cntStr);
    }

    // index
    // pinVCC
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    // pincPLS
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    // pinDir
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    // pinEnable
    // '0','1','2','3', '4', '5', '6', '7', '8' '9' '10', '10', '11', '12', '13' 'A0', 'A1', 'A2', 'A3', 'A4', 'A5'
    ManyKitArduino.prototype.StepMotoInit = function (index, pinVCC, pincPLS, pinDir, pinEnable) {
        var self = this;
        
        // OT_STEPMOTO_I
        //var cntStr = "51" + "," + index + "," + pinVCC + "," + pincPLS + "," + pinDir + "," + pinEnable;
        var cntStr = "0000" + "51" + " " + index + " " + pinToArduino(pinVCC) + " " + pinToArduino(pincPLS) + " " + pinToArduino(pinDir) + " " + pinToArduino(pinEnable) + "\n";
        self.writeData(cntStr);
    }

    ManyKitArduino.prototype.StepMotoEnable = function (index, enable) {
        var self = this;
        
        // OT_STEPMOTO_ENABLE
        //var cntStr = "52" + "," + index + "," + enable;
        var cntStr = "0000" + "52" + " " + index + " " + boolToArduino(enable) + "\n";
        self.writeData(cntStr);
    }

    ManyKitArduino.prototype.StepMotoDir = function (index, forward) {
        var self = this;
        
        // OT_STEPMOTO_DIR
        //var cntStr = "53" + "," + index + "," + forward;
        var cntStr = "0000" + "53" + " " + index + " " + boolToArduino(forward) + "\n";
        self.writeData(cntStr);
    }

    ManyKitArduino.prototype.StepMotoStep = function (index, delay) {
        var self = this;

        // OT_STEPMOTO_STEP
        //var cntStr = "54" + "," + index + "," + delay;
        var cntStr = "0000" + "54" + " " + index + " " + delay + "\n";
        self.writeData(cntStr);
    }

    ManyKitArduino.prototype.sayHelloToTellAddress = function () {
        var self = this;

        self.pinMode("5", "OUTPUT"); // send this to give udp ip & port to esp; important!!!
    }

}