function Decoder(bytes, port) {
  // Decode an uplink message from a buffer
  var fields = [['distance 1', 2],['distance 2', 2],['distance 3', 2],['timestamp', 5],['voltage', 2],["temp", 2],[]];
  //port = 3;
  var available_fields = port.toString(2);
  available_fields = available_fields.split("").reverse().join("");
  available_fields = available_fields + "0000000".substr(available_fields.length);
  var payloadByteNext = 0;
  var output = {};
  payloadByteNext;
  
  for(var i = 0; i < available_fields.length; i++){
    if(available_fields.charAt(i) == "1"){
      if(fields[i][0] == 'timestamp'){
        var timestamp = "";
        for(var timeIndex =  fields[i][1] -1 ; timeIndex > -1; timeIndex--){
          //timestamp += String.fromCharCode(bytes[i]);
          timestamp += String(bytes[timeIndex + payloadByteNext]);
          timestamp += ":";
          }
        output['timetamp'] = timestamp;
        payloadByteNext += fields[i][1];
         
      }else if(fields[i][0] == "distance 1" || fields[i][0] == "distance 2" || fields[i][0] =="distance 3"){
        for(var history = 0; history < 3; history++){
          value = (bytes[payloadByteNext]);
          value |= (bytes[payloadByteNext + 1] << 8);
          if(history === 0){name_field = fields[i][0];}else{name_field = fields[i][0] +  " hist_"  + String(history);}
          output[name_field] = value ;  
          payloadByteNext += fields[i][1];
        }
      }else{
         value = bytes[payloadByteNext] ;
        
        for(var j = 1; j < fields[i][1]; j++){
          value |= (bytes[payloadByteNext + j] << (8 * j));
        }
        name_field = fields[i][0];
        output[name_field] = value;
        payloadByteNext += fields[i][1];
    }
    }
  }
  
 output["voltage"] = (output["voltage"] * 2 * 3.3) / 1024;
  return output;
}
