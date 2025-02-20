from flask import Flask, render_template, request

app = Flask(__name__)

# Route to render the homepage (your joystick and button UI)
@app.route('/')
def home():
    return render_template('index.html')

# Route to handle joystick control (x, y axis values)
@app.route('/control', methods=['GET'])
def control():
    x_value = request.args.get('x', type=int)  # Get the x value from the request
    y_value = request.args.get('y', type=int)  # Get the y value from the request

    if x_value is not None and y_value is not None:
        # Process joystick control values (you can send them to the robot here)
        print(f"Joystick moved to x: {x_value}, y: {y_value}")
        return f"Joystick moved to x: {x_value}, y: {y_value}", 200
    return "Invalid joystick data", 400

# Route for the "function1" button press
@app.route('/function1', methods=['GET'])
def function1():
    # Handle the function 1 button press here (send command to robot, etc.)
    print("Function 1 triggered")
    return "Function 1 triggered", 200

# Route for stopping the robot
@app.route('/stop', methods=['GET'])
def stop():
    # Handle stop action here (e.g., send stop command to robot)
    print("Robot stopped")
    return "Robot stopped", 200

# Run the app
if __name__ == '__main__':
    app.run(debug=True, host="0.0.0.0", port=3000)
