# Image-Processing-Microprocessor
This program was made to track retroreflective tape in a set shape using infared light and cameras.

# Socket Server
Since this program was ran on a separate board (NVIDIA Jetson TX1) than the main one (Roborio), they
had to connect through ethernet. Although the Socket is messy and a little buggy, it served its purpose
and had relatively quick connection between the two boards. The only error rising from this is when the 
system would temporarily turn off and on, causing the socket server to have issues due to this loss of connection.

# OpenCV 
By utilizing OpenCv's contour detection, I matched like shapes together using a pretty simple sorting algorithm, 
finding the biggest shape on screen that is IR, and finding its closest match. The OpenCV ran through MANY iterations, 
as I found new solutions and tested new setups. 

Issues: 

  -Sunlight is a big issue for this as it is all infared. Ideally, this program would run indoors.
  However, you can adjust camera settings such as Saturation to account for this lighting change.
  
  -Making sure each USB was assigned the right hardware value (0 or 1). I found that keeping them plugged into
  the same port every time fixed it, but can still prove to be an issue if there is a USB hub attached.
