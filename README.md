# RAB2-CO2 and RDK3 Code Example

Rutronik Development Kit Programmable System-on-Chip CY8C6245AZI-S3D72 "RAB2-CO2 & RDK3" Code Example. 

This code example demonstrates the PAS CO2 and SCD41 sensors with the RDK3.

 <img src="images/rab2co2model.jpg" style="zoom:20%;" />

## Requirements

- [ModusToolbox® software](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software/) v2.4

### Using the code example with a ModusToolbox IDE:

1. Import the project: **File** > **Import...** > **General** > **Existing Projects into Workspace** > **Next**.
2. Select the directory where **"RDK3_RAB2_CO2_Adapter_Demo"** resides and click  **Finish**.
3. Update libraries using  a **"Library Manager"** tool.
4. Select and build the project **Project ** > **Build Project**.

### Operation

The firmware example uses KitProg3 Debug UART for the debug output. The results are read from the sensor and are printed on the terminal window.

<img src="images/rab2co2_info.png" style="zoom:100%;" />

### Debugging

If you successfully have imported the example, the debugging configurations are already prepared to be used with a the KitProg3, MiniProg4, or J-link. Open the ModusToolbox perspective and find the Quick Panel. Click on the desired debug launch configuration and wait for the programming to  complete and debugging process to start.

<img src="images/debugging.png" style="zoom:100%;" />

## Legal Disclaimer

The evaluation board including the software is for testing purposes only and, because it has limited functions and limited resilience, is not suitable for permanent use under real conditions. If the evaluation board is nevertheless used under real conditions, this is done at one’s responsibility; any liability of Rutronik is insofar excluded. 

<img src="images/rutronik_origin_kaunas.png" style="zoom:50%;" />



