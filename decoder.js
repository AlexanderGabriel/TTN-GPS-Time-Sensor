function Decoder(bytes, port) {
    var decoded = {};
  
   
      var payload = String.fromCharCode.apply(null, bytes);
      decoded.latitude = parseFloat(payload.substr(0,10));
      decoded.longitude= parseFloat(payload.substr(10,10));
      decoded.altitude = parseFloat(payload.substr(20,6));
      decoded.hdop = parseFloat(payload.substr(26,4));
  
      
    return decoded;
  }