from flask import Flask, render_template, Response, send_file,request,jsonify
import cv2
import threading
import numpy as np
import time
import requests
from waitress import serve
import socket


app = Flask(__name__)
camera = None
is_camera_running = False
latest_frame = None
latest_captured_image = None
latest_processed_image = None
product_count = 0
lock = threading.Lock()
event = threading.Event()
esp32_ip = "192.168.43.157"  # Replace with your ESP32 IP address



#serverIP = "192.168.0.133"
serverPort = 1234

# Connect to the ESP32 server
clientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
clientSocket.connect((esp32_ip, serverPort))
print("Connected to ESP32 server")

#serverSocket.listen(1)
#print("Waiting for ESP32 connection...")
"""ef send_to_esp():
    set_value = 10
    while True:
        if Set_Servo_On == 1:
            dataToEsp = "Turn on Servo"
            clientSocket.sendall(dataToEsp.encode())
            print("Data sent from Python to SERVO")"""

"""def Receive_motor():
    global Motor_value
    receivedData = clientSocket.recv(1024).decode()
    #if not receivedData:
    #    break
    
    print("Received data from ESP32: " + receivedData)

    if receivedData.startswith("Motor:"):
        Motor_value = int(receivedData.split(":")[1].strip())
        print("Motor value",Motor_value)"""

def blink_led_count():
    global Total_product_count
    dataToLED = f"LED:{Total_product_count}"
    clientSocket.sendall(dataToLED.encode())
    print("LED count start")
    
    
def Servo_on():
    global Set_Servo_On
    if Set_Servo_On == 1:
            dataToEsp = "Turn on Servo"
            clientSocket.sendall(dataToEsp.encode())
            print("Servo on")
            #update_servo_status()
            time.sleep(7.5)
            Set_Servo_On = 0

def send_humidity_to_esp32():
    global Humi_Set_Limit
    if Humi_Set_Limit == 1:
        #dataToTemp = f"{min_temp},{max_temp}"
        dataToHumi = f"Humidity:{min_humi},{max_humi}"
        #time.sleep(1)
        clientSocket.sendall(dataToHumi.encode('utf-8'))
        print("Humi sent")
        Humi_Set_Limit == 0 
    
def send_data_to_esp32():
    global Temp_Set_Limit,Set_Servo_On
    if Temp_Set_Limit == 1:
        #dataToTemp = f"{min_temp},{max_temp}"
        dataToTemp = f"Temperature:{min_temp},{max_temp}"
        #time.sleep(1)
        clientSocket.sendall(dataToTemp.encode('utf-8'))
        print("temp sent")
        Temp_Set_Limit == 0

    

    """if Set_Servo_On == 1:
            dataToEsp = "Turn on Servo"
            clientSocket.sendall(dataToEsp.encode())
            print("Servo on")
            Set_Servo_On = 0"""
   

        #return None

    #clientSocket.close()
    #print("Connection closed")
    
"""def Show_Humidity():
    receivedData = clientSocket.recv(1024).decode()
    #if not receivedData:
        #break
     
    #print("Received data from ESP32: " + receivedData)

    if receivedData.startswith("Humidity:"):
            Hum_value = receivedData.split(":")[1].strip()
            print("Humidity Value:",Hum_value)"""
"""def receive_ir_data():
    while True:
        receivedData = clientSocket.recv(1024).decode()
        if not receivedData:
            break
        print("Received data from ESP32: " + receivedData)

        if receivedData.startswith("IR Value:"):
            ir_sensor_value = receivedData.split(":")[1].strip()
            print("ir sensor value",ir_sensor_value)
            print("mintemp",min_temp)
            print("maxtemp",max_temp)
            
            if ((ir_sensor_value == "1") and (min_temp <= Temp_value <= max_temp) and (min_humi <= Hum_value <= max_humi)):
            #if ((Temp_value >= min_temp & Temp_value <= max_temp) and (Hum_value >= min_humi & Hum_value <= max_humi)):

                print("IR Sensor triggered")
                #print("IR Value:",ir_sensor_value)
                #event.set()
                #time.sleep(4)
                capture_image()"""
        
    
def esp32_communication():

    global Temp_Set_Limit,Temp_value,Hum_value,min_temp,max_temp,min_humi,max_humi,Start_Inspection,Motor_value
    #set_value = 10
    Temp_value = 0
    Hum_value = 0
    min_temp = 0
    max_temp = 0
    min_humi = 0
    max_humi = 0
    Start_Inspection = 0
    min_temp = float(min_temp) 
    max_temp = float(max_temp)
    min_humi = float(min_humi)
    max_humi = float(max_humi)
    Temp_value = float(Temp_value)
    Hum_value = float(Hum_value)
    
    #Send data to the ESP32
    #dataToSend = "Hello ESP32"
    #clientSocket.sendall(dataToSend.encode())
    #print("Data sent to ESP32")
    """if Total_product_count >= set_value:
            dataToEsp = "Turn on Servo"
            clientSocket.sendall(dataToEsp.encode())
            print("Data sent from Python to ESP32")"""
    # Send data from Python to ESP32
    """if Temp_Set_Limit == 1:  
         dataToTemp = "Minimum Temperature:" + str(min_temp)
         clientSocket.sendall(dataToTemp.encode())
         print("Min temp sent")

         dataToHum = "Maximum Temperature:" + str(max_temp)
         clientSocket.sendall(dataToHum.encode())
         print("Max temp sent")
         Temp_Set_Limit = 0"""

     


    # Receive and print data from the ESP32 continuously
    while True:
        #print(Total_product_count)
        #print(Set_Servo_On)
        #clientSocket, clientAddress = serverSocket.accept()
        #print("Connected to ESP32 at:", clientAddress)

        """if Set_Servo_On == 1:
                dataToEsp = "Turn on Servo"
                clientSocket.sendall(dataToEsp.encode())
                print("Data sent from Python to SERVO")"""
                
      
                
        receivedData = clientSocket.recv(1024).decode()
        if not receivedData:
            break
        print("Received data from ESP32: " + receivedData)

        if receivedData.startswith("IR Value:"):
            ir_sensor_value = int(receivedData.split(":")[1].strip())
            print("ir sensor value",ir_sensor_value)
            #print("mintemp",min_temp)
            #print("maxtemp",max_temp)
            #print("actualtemp",Temp_value)
            #print("actualhum",Hum_value)
            
            if ((ir_sensor_value == 1) and (min_temp <= Temp_value <= max_temp) and (min_humi <= Hum_value <= max_humi) and (Start_Inspection == 1)) :
            #if ((Temp_value >= min_temp & Temp_value <= max_temp) and (Hum_value >= min_humi & Hum_value <= max_humi)):

                print("IR Sensor triggered")
                #print("IR Value:",ir_sensor_value)
                #event.set()
                #time.sleep(4)
                capture_image()

        if receivedData.startswith("Motor:"):
            Motor_value = int(receivedData.split(":")[1].strip())

        elif (min_temp <= Temp_value <= max_temp):
            Motor_value = 0
            
            #print("Motor value",Motor_value)

        
        #Show_Humidity()

        # Split the 'sendData' string into parts based on commas (',')
        data_parts = receivedData.split(',')
        print(data_parts)

        for data_part in data_parts:
            if 'Temperature' in data_part:
                Temp_value = float(data_part.split(':')[1].strip())
                print("Temperature Value:",Temp_value)
            elif 'Humidity' in data_part:
                humidity_data = data_part.split('n')[0]
                Hum_value = float(humidity_data.split(':')[1].strip())
                print("Humidity Value:",Hum_value)


        


            """elif receivedData.startswith("IR Value:"):
                ir_sensor_value = receivedData.split(":")[1].strip()
                print("ir sensor value",ir_sensor_value)
                print("mintemp",min_temp)
                print("maxtemp",max_temp)
            
                if ((ir_sensor_value == "1") and (min_temp <= Temp_value <= max_temp) and (min_humi <= Hum_value <= max_humi)):
                #if ((Temp_value >= min_temp & Temp_value <= max_temp) and (Hum_value >= min_humi & Hum_value <= max_humi)):

                    print("IR Sensor triggered")
                    #print("IR Value:",ir_sensor_value)
                    #event.set()
                    #time.sleep(4)
                    capture_image()"""

         # Process the received sensor values
        """if receivedData.startswith("IR Value:"):
            ir_sensor_value = receivedData.split(":")[1].strip()
            if ir_sensor_value == "1":
                print("IR Sensor triggered")
                print("IR Value:",ir_sensor_value)
                #event.set()
                capture_image()
        #elif receivedData.startswith("Temperature:"):
            Temp_value = receivedData.split(":")[1].strip()
            print("Temperature Value:",Temp_value)

        elif receivedData.startswith("Humidity:"):
            Hum_value = receivedData.split(":")[1].strip()
            print("Humidity Value:",Hum_value)"""
        
        #Temp_Set_Limit == 1  
                
        #elif Temp_Set_Limit == 1:  
            # dataToTemp = "Minimum Temperature:" + str(min_temp)
             #clientSocket.sendall(dataToTemp.encode())
             #print("Min temp sent")
             #dataToTemp = "Maximum Temperature:" + str(max_temp)
             #clientSocket.sendall(dataToTemp.encode())
             #print("Max temp sent")
             
             #Temp_Set_Limit = 0
                
         
                
        #time.sleep(1) 

       
    # Close the connection
    clientSocket.close()
    print("Connection closed")

# Set the maximum FPS for video feed
#MAX_FPS = 40
# Set the resized frame dimensions
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
#RESIZED_WIDTH = 320
#RESIZED_HEIGHT = 240


def init_camera():
    global camera
    if camera is None:
        #camera = cv2.VideoCapture("http://10.48.8.66:8080/video")
        camera = cv2.VideoCapture(1)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)


def release_camera():
    global camera
    if camera is not None:
        camera.release()
        camera = None


def count_products(image):
    # Implement your product counting algorithm here
    # Convert the image to the HSV color space

    # Convert the image to the HSV color space
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #lower_color = np.array([0, 0, 200])  # Lower color threshold for white (low saturation, high value)
    #upper_color = np.array([255, 30, 255])  # Upper color threshold for silver (high saturation, high value)

    #mask = cv2.inRange(hsv, lower_color, upper_color)

    # Apply GaussianBlur to reduce noise and improve circle detection
    blurred_image = cv2.GaussianBlur(gray_image, (9, 9), 2)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(blurred_image, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)



    # Detect circles using Hough Circle Transform
    circles = cv2.HoughCircles(
        mask,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=10,
        param1=40,
        param2=25,
        minRadius=5,
        maxRadius=25
    )

    # Initialize product count
    product_count = 0

    if circles is not None:
        # Convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")

        for (x, y, r) in circles:
            # Ensure the circle coordinates are within the valid image boundaries
            if 0 <= y < image.shape[0] and 0 <= x < image.shape[1]:
                # Check if the circle is white (you can adjust this condition)
                if image[y, x][0] >= 200 and image[y, x][1] >= 200 and image[y, x][2] >= 200:
                    # Draw the circle on the original image (optional)
                    # cv2.circle(image, (x, y), r, (0, 255, 0), 4)

                    # Increment the product count
                    product_count += 1

        # Display the image with the count information (optional)
        # cv2.putText(image, f'Product Count: {product_count}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        # cv2.imshow('Counting Result', image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

    # Return the final product count
    return product_count
    """hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper color thresholds for the product
    lower_color = np.array([0, 0, 200])  # Lower color threshold for white (low saturation, high value)
    upper_color = np.array([255, 30, 255])  # Upper color threshold for silver (high saturation, high value)

    #lower_color = np.array([0, 0, 0])  # Lower color threshold for black (low saturation, low value)
    #upper_color = np.array([255, 255, 30]) 
 
    # Apply color thresholding to isolate the product
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Perform morphological operations to remove noise and enhance the mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize product count
    product_count = 0

    # Iterate through the contours and count the products
    for contour in contours:
        # Calculate the contour area
        area = cv2.contourArea(contour)

        # Filter out small contours to eliminate noise
        if area > 15:
            # Find the convex hull of the contour
            hull = cv2.convexHull(contour)

            # Calculate the perimeter of the convex hull
            perimeter = cv2.arcLength(hull, True)

            # Calculate the circularity of the contour
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            
            # Print the circularity
            #print(f"Circularity: {circularity}")

            # Filter out elongated and irregular contours based on circularity
            if 0.25 <= circularity <= 1.4:
                # Draw the contour on the image
                #cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)

                # Increment the product count
                product_count += 1

    # Display the image with the count information
    #cv2.putText(image, f'Product Count: {product_count}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    #cv2.imshow('Counting Result', image)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    # Return the final product count
    return product_count"""





def capture_frames():
    global camera, is_camera_running, latest_frame
    while is_camera_running:
        success, frame = camera.read()
        if success:
            #resized_frame = cv2.resize(frame, (RESIZED_WIDTH, RESIZED_HEIGHT))
            with lock:
               # latest_frame = resized_frame
                latest_frame = frame.copy()
            event.set()


def generate_frames():
    global latest_frame, product_count
    while True:
        event.wait()
        event.clear()

        with lock:
            if latest_frame is None:
                continue

            frame = latest_frame.copy()

        # Perform product counting on the frame
        product_count = count_products(frame)

        # Draw the product count on the frame
        cv2.putText(frame, f'Product Count: {product_count}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Convert the frame to JPEG format
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

def generate_contours_frames():
    global latest_frame
    while True:
        event.wait()
        event.clear()

        with lock:
            if latest_frame is None:
                continue

            frame = latest_frame.copy()

        # Perform product counting on the frame
        product_count = count_products(frame)

        # Find contours in the mask
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_color = np.array([0, 0, 200])
        upper_color = np.array([255, 30, 255])
        mask = cv2.inRange(hsv, lower_color, upper_color)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw the contours on the frame
        cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)

        # Convert the frame to JPEG format
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

Total_product_count = 0
Set_Servo_On = 0
Set_Product_Count = 0
def capture_image():
    global latest_frame, latest_captured_image,Total_product_count,Set_Servo_On,Set_Product_Count
    set_value = int(Set_Product_Count)
    with lock:
        if latest_frame is not None:
            latest_captured_image = latest_frame.copy()
            cv2.imwrite('captured_image.jpg', latest_captured_image)

   
    if latest_captured_image is not None:
    #while latest_captured_image is not None:
        product_count = count_products(latest_captured_image)
        #count()
        Total_product_count += product_count  # Accumulate the product count
        if Total_product_count >= set_value:
            print(f"Product Count: {set_value}")
        else:
            print(f"Product Count: {Total_product_count}")

        # Check if the product count has reached the set value
        if Total_product_count >= set_value:
            print("Resetting product count.")
            time.sleep(1.5) 
            Total_product_count = 0# Reset the count
            Set_Servo_On = 1
            #send_data_to_esp32()
            Servo_on()
        else:
            Set_Servo_On = 0
            #break           

    event.set()
    #print('Image captured')
    
#Total_product_count = 0
#while True:
def count():
    #Total_product_count = 0  # Initialize total product count outside the loop
    global Total_product_count
    set_value = 10  # Specify the value to reset the count

    while latest_captured_image is not None:
        
        #product_count = count_products(latest_captured_image)
        Total_product_count = product_count + Total_product_count   # Accumulate the product count

        print(f"Product Count: {Total_product_count}")


        # Check if the product count has reached the set value
        if Total_product_count >= set_value:
            print("Resetting product count.")
            Total_product_count = 0# Reset the count
            #product_count = 0
            break
            

#capture_thread = threading.Thread(target=capture_image)
#capture_thread.start()

# Call esp32_communication() to start the communication
#esp32_communication()

def process_image():
    global latest_captured_image, latest_processed_image
    captured_image = cv2.imread('captured_image.jpg')
    if captured_image is not None:
        processed_image = captured_image.copy()
        # Perform any additional processing on the captured image
        # ...
        latest_processed_image = processed_image
        cv2.imwrite('processed_image.jpg', latest_processed_image)
        print('Image processed')


@app.route('/')
def index():
    #return render_template('temperature_graph.html')
    return render_template('welcome.html')
    #return render_template('trial.html')
@app.route('/system-monitor')
def system_monitor():
    # Render the system monitor and control page HTML
    return render_template('system-monitor.html')

@app.route('/contours_feed')
def contours_feed():
    return Response(generate_contours_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')



@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/capture_image', methods=['GET', 'POST'])
def capture_image_route():
    threading.Thread(target=capture_image).start()
    return 'Image capture process started'


@app.route('/process_image', methods=['GET', 'POST'])
def process_image_route():
    threading.Thread(target=process_image).start()
    return 'Image processing started'


@app.route('/stop_camera')
def stop_camera():
    global is_camera_running
    is_camera_running = False
    event.set()
    release_camera()
    return 'Camera stopped'


@app.route('/start_camera')
def start_camera(): 
    global is_camera_running
    init_camera()
    is_camera_running = True
    threading.Thread(target=capture_frames).start()
    return 'Camera started'


@app.route('/get_product_count')
def get_product_count():
    #global product_count
    global Total_product_count

    if (Total_product_count >= int(Set_Product_Count)):
       return str(Set_Product_Count)
    else:
       return str(Total_product_count)
        

Temp_Set_Limit = 0
@app.route('/set_temperature',methods=['POST'])
def set_temperature():
    global min_temp,max_temp,Temp_Set_Limit
    data = request.get_json()
    min_temp = float(data.get('min'))
    max_temp = float(data.get('max'))

    # Now you have the min_temp and max_temp values in Python
    # You can process them as needed
    print("Minimum Temperature:", min_temp)
    print("Maximum Temperature:", max_temp)
    """min_temp = int(request.form['min'])
    max_temp = int(request.form['max'])
    #min_temp = request.args.get('min')
    #max_temp = request.args.get('max')"""
    Temp_Set_Limit = 1
    send_data_to_esp32()

    return 'Temperature values set successfully'

Humi_Set_Limit = 0
@app.route('/set_humidity',methods=['POST'])
def set_humidity():
    global min_humi,max_humi,Humi_Set_Limit
    data1 = request.get_json()
    min_humi = float(data1.get('min'))
    max_humi = float(data1.get('max'))

    # Now you have the min_humi and max_humi values in Python
    # You can process them as needed
    print("Minimum Humidity:", min_humi)
    print("Maximum Humidity:", max_humi)
    Humi_Set_Limit = 1
    send_humidity_to_esp32()
    

    # Forward the temperature values to the Arduino
    #arduino_url = f'http://{arduino_ip}/set_temperature?min={min_temp}&max={max_temp}'
    #response = requests.get(arduino_url)
    
    return 'Humidity values set successfully'


@app.route('/set_productcount',methods=['POST'])
def set_productcount():
    global Set_Product_Count
    data2 = request.get_json()
    Set_Product_Count = data2.get('productcount')
    

    # Now you have the min_humi and max_humi values in Python
    # You can process them as needed
    print("Set Product count:", Set_Product_Count)
    
    return 'Product Count set successfully'



@app.route('/get_temperature_and_humidity')
def get_temperature_and_humidity():
    global Temp_value, Hum_value
   
    return jsonify(temperature=Temp_value, humidity=Hum_value)

"""servo_status = False
# Function to continuously update the servo_status based on Set_Servo_On
def update_servo_status():
    global Set_Servo_On, servo_status
    #while True:
    if Set_Servo_On == 1:
        servo_status = True
        print("servo status",servo_status)
        time.sleep(7.5)
        servo_status = False
            #Set_Servo_On = 0
    return 'Servo status updated'
        #time.sleep(1) """ # Adjust the interval based on your requirement

# Start the thread to continuously update the servo_status
"""thread = threading.Thread(target=update_servo_status)
thread.daemon = True
thread.start()"""

# Add a new route to get the current status of the servo motor

@app.route('/get_servo_status')
def get_servo_status_route():
    global Set_Servo_On
    #global servo_status
    #servo_status = False
    """if Set_Servo_On == 1:
        servo_status = True
        time.sleep(6)
        servo_status = False"""
    #print("servo status",Set_Servo_On)
    return jsonify({'servoStatus': Set_Servo_On})

Motor_value = 0
@app.route('/get_DCmotor_status')
def get_DCmotor_status_route():
    global Motor_value
    #if Motor_value == 1:    
    #print("DC motor status",Motor_value)
    return jsonify({'MotorStatus': Motor_value})

    """else:
        Motor_value = 0
        return jsonify({'MotorStatus': Motor_value})"""
        
    

Start_Inspection = 0
@app.route('/startinspection')
def startinspection():
    global Start_Inspection

    Start_Inspection = 1
    return 'Inspection Started'


@app.route('/stopinspection')
def stopinspection():
    global Start_Inspection

    Start_Inspection = 0
    return 'Inspection Stopped'

@app.route("/blink")
def blink_led():
    count = Total_product_count
    if count > 0:
        blink_led_count()
        #response = requests.get(f"http://{esp32_ip}/?command=BLINK&count={count}")
        #return response.text
        return "valid product count."
    else:
        return "Invalid product count."


@app.route('/latest_captured_image')
def latest_captured_image_route():
    global latest_captured_image
    if latest_captured_image is not None:
        cv2.imwrite('latest_captured_image.jpg', latest_captured_image)
        return send_file('latest_captured_image.jpg', mimetype='image/jpeg')
    else:
        return 'No captured image available'


@app.route('/latest_processed_image')
def latest_processed_image_route():
    global latest_processed_image
    if latest_processed_image is not None:
        cv2.imwrite('latest_processed_image.jpg', latest_processed_image)
        return send_file('latest_processed_image.jpg', mimetype='image/jpeg')
    else:
        return 'No processed image available'


if __name__ == '__main__':
    #init_camera()
    #is_camera_running = True
    #threading.Thread(target=capture_frames).start()
    threading.Thread(target=esp32_communication).start()
    threading.Thread(target=send_data_to_esp32).start()
    threading.Thread(target=capture_image).start()
    serve(app, host='0.0.0.0', port=5000)

