#MSG TYP RadarDistancemsg

Header header    # timestamp in the header is createtime of message
time sampleTime  # timestamp of Sample. Create by Sensor
uint32 sensorID  # Sensor ID
float32 fovHorizontal # the size of the arc that the distance reading is valid for [rad] A111 80° = 1.3962634015955
float32 fovVertical  # the size of the arc that the distance reading is valid for [rad] A111 40° = 0.69813170079773
uint32 numbersOfBins  # Numbers of Bins between start and end
float32 lengthOfBin   # Physical Size of a Bin in [meter]
float32 startDistance # Start point of scan in [meter]
float32 endDistance   # EndPoint point of scan in [meter]
uint32[] DistanceBins
float32[] PhaseBins


