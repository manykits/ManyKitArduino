// manykitudp.js

var dgram = require('dgram');
ManyKitUDP = module.exports = function (url, port) {
    var self = this;

    self.url = url;
    self.port = port;
    self.FunOnData = null;

    console.log("url:" + self.url);
    console.log("port:" + self.port);  

    self.udp = dgram.createSocket('udp4');

    self.udp.on('message', function (msg, rinfo) {
        if (null != self.FunOnData)
        {
            self.FunOnData(msg);
        }
    });
    self.udp.on('close', function () {
        console.log('this.udp close');
    });

    ManyKitUDP.prototype.setFunOnData = function (fun) {
        var self = this;
        self.FunOnData = fun;
    };

    ManyKitUDP.prototype.writeUDP = function (address, port, data) {
        var self = this;

        self.udp.send(data, 0, data.length, port, address, function (err, bytes) {
            if (err)
                console.log('clientDist upd send err');
            else {
                //console.log('clientDist upd send %d bytes',bytes);
            }
        });
    }
}