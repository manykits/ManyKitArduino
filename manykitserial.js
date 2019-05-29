// manykitserial.js

function toBuffer(ab) {
    var buffer = new Buffer(ab.byteLength);
    var view = new Uint8Array(ab);
    for (var i = 0; i < buffer.length; ++i) {
        buffer[i] = view[i];
    }
    return buffer;
}

function buffer2ArrayBuffer(buffer) {
    var buf = new ArrayBuffer(buffer.length);
    var bufView = new Uint8Array(buf);
    for (var i = 0; i < buffer.length; i++) {
        bufView[i] = buffer.charCodeAt(i);
    }
    return buf;
}

var SerialPort = require('serialport');
//var chromeserial = require('chrome')
ManyKitSerial = module.exports = function (port, baudrate) {
    var self = this;

    self.Port = port;
    self.Baudrate = baudrate;

    self.FunOnOpen = null;
    self.FunOnClose = null;
    self.FunOnDisconnect = null;
    self.FunOnError = null;
    self.FunOnData = null;

    self.serialPort = null;
    self.serialPort = new SerialPort(port, baudrate);
    //self.chromt = new chrome.serial;
    self.connectionId = 0;
    if (null != self.chromePort)
    {
        self.chromePort.connect(port,
            {
                bitrate: baudrate
            }, function (connectionInfo) {
                var connectionId = connectionInfo.connectionId;
                self.connectionId = connectionId;
            })
    }

    ManyKitSerial.prototype.setFunOnOpen = function (fun) {
        self.FunOnOpen = fun;

        if (null != self.serialPort && null != fun)
        {
            self.serialPort.on("open", fun);
        }
    };

    ManyKitSerial.prototype.setFunOnClose = function (fun) {
        self.FunOnClose = fun;

        if (null != self.serialPort && null != fun) {
            self.serialPort.on("close", fun);
        }
    };

    ManyKitSerial.prototype.setFunOnDisconnect = function (fun) {
        self.FunOnDisconnect = fun;

        if (null != self.serialPort && null != fun) {
            self.serialPort.on("disconnect", fun);
        }
    };

    ManyKitSerial.prototype.setFunOnError = function (fun) {
        self.FunOnError = fun;

        if (null != self.serialPort && null != fun) {
            self.serialPort.on("error", fun);
        }
    };

    ManyKitSerial.prototype.setFunOnData = function (fun) {
        self.FunOnData = fun;

        if (null != self.serialPort && null != fun) {
            self.serialPort.on("data", fun);
        }

        if (null != self.chromePort && null!=fun)
        {
            self.chromePort.onReceive.addListener(function recvCallback(info) {
                var connectionId = info.connectionId;
                fun(toBuffer(info.data));
            }.bind(this));
        }
    };

    ManyKitSerial.prototype.close = function () {
        if (null != self.serialPort) {
            self.serialPort.close();
        }

        if (null != self.chromePort && null != self.connectionId)
        {
            self.chromePort.disconnect(self.connectionId, function () {
            });
        }
    }

    ManyKitSerial.prototype.write = function (dataStr) {
        if (null != self.serialPort) {
            self.serialPort.write(dataStr, function(err){
                if (err)
                {
                    return console.log('Error on write: ', err.message);
                }
            });
        }

        if (null != self.chromePort && null != self.connectionId){
            self.chromePort.send(self.connectionId, buffer2ArrayBuffer(dataStr), function () {
                //self.chromePort.flush(self.connectionId, function (result) {
                //});
            });
        }
    };

}