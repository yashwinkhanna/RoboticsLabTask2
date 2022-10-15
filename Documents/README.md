# RoboticsLabTask2
41014 Industrial Robotics - Lab Assignment 2

## Robot Pancake Chef

### Description:
The Robot Chef is a robot that can cook and plate meals using an expandable, user-input menu. For our project, we will teach it how to make pancakes using the UR3 to grab ingredients and cooking utensils, and a new robot that will dispense liquid ingredients including pancake batter and syrup. It is intended to be used in a commercial or home kitchen, where the user can select and customise their meal via a touchscreen interface. With the potential for versatility beyond pancakes and a capability for storage, both robots will travel on linear rails to assist the kitchen process. 

### New Industrial Robot:
The new robot will be a 3-link arm, with a nozzle for an end-effector with the purpose to dispense different liquid ingredients. These ingredients will be connected via tubing to various bottles allowing the robot to coordinate between them. The arm will be mounted on a linear rail, allowing translation along the countertop including towards a stove or to be stored away. Dynamically, the robot will have 3 DOF, providing rotation in x, y & z axes. Supplementing this movement, links 1 and 2 will be approximately 300 mm and link 3 closest to the nozzle is estimated at 150 mm. 

### Chosen New Robot: IRB 910SC SCARA
![image](https://user-images.githubusercontent.com/48670096/194475642-0c031f23-0073-4a33-a118-db747b870759.png)
![image](https://user-images.githubusercontent.com/48670096/194475548-773d11e1-8d8f-44eb-bd44-ef9637764abe.png)

- Manufacturer: ABB
- DOF: 3
- Payload: up to 3kg
- Range: 0.55m

A SCARA (Selective Compliance Articulated Robot Arm) was chosen as its characteristics best suit our application. In particular, in combination with the versatility of the 7 DOF Linear UR3, the supplementing device is not required access to the entire 3D space. Rather, we can employ a device with fewer degrees of freedom that is more focused within the kitchen benchtop space and one that is more specialised for this use case. In doing so, a 4-link device greatly simplifies SafeCo's assignment through reducing the necessary programming complexity and computational power. Secondly, SCARA robots specialise in material handling and dispensing, ideal for our described application. Their pivoting and vertical actuation movements are tailored to the perceived design, where the device can comfortably execute movements to and from the stovetop without demanding significant benchspace.

Specific to the IRB 910, its technical specifications in payload and range align comfortably within our design, in addition to allowing excess capability for potential safety measures. The device is available with a food-safe cover, protecting any fumes or particles penetrating in our out of the device. With an IP20 rating, the 910 is also safe for exposure to liquids - a prevalent safety risk in the kitchen environment.

Ultimately, the characteristics of a SCARA device applied with the technical capabilities of the IRB 910SC make it an ideal choice for the Robot Chef.

