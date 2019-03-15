# ManyKitArduino
javascript manykit arduino firewall communicate
# sample:

var ManyKitArduino = require("./common/manykitarduino");

var mkArduino = new ManyKitArduino("COM3");

mkArduino.pinMode("13", "OUTPUT");

var isHigh = false;

function myFunc() {

  var isReady = mkArduino.isBoardReady();
  
  if (isReady) {
  
    var val = mkArduino.digitalRead("13");

    if (!isHigh) {
    
      console.log("high");
      
      mkArduino.digitalWrite("13", "high");
      
      isHigh = true;
      
    }
    
    else {
    
      console.log("low");
      
      mkArduino.digitalWrite("13", "low");
      
      isHigh = false;
      
    }
    
  }
  
}

setInterval(myFunc, 1000);
