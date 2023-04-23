## TeamCode Module

Welcome!

This is the TeamCode module for the project of Pascal Eagles for the FTC - 2023. Here you 
will be able to find sample code from the FtcRobotController repository as well as the template
used by the Pascal Eagles team in 2023. Feel free to download or fork this repository 
and modify it for your robot. 

## Creating your own OpModes

The easiest way to create your own OpMode is to copy a Sample OpMode and make it your own.

Sample opmodes exist in the FtcRobotController module.
To locate these samples, find the FtcRobotController module in the "Project/Android" tab.

Expand the following tree elements:
 FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples

### Naming of Samples

To gain a better understanding of how the samples are organized, and how to interpret the
naming system, it will help to understand the conventions that were used during their creation.

These conventions are described (in detail) in the sample_conventions.md file in this folder.

To summarize: A range of different samples classes will reside in the java/external/samples.
The class names will follow a naming convention which indicates the purpose of each class.
The prefix of the name will be one of the following:

Basic:  	This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones examples.

Sensor:    	This is a Sample OpMode that shows how to use a specific sensor.
            It is not intended to drive a functioning robot, it is simply showing the minimal code
            required to read and display the sensor values.

Robot:	    This is a Sample OpMode that assumes a simple two-motor (differential) drive base.
            It may be used to provide a common baseline driving OpMode, or
            to demonstrate how a particular sensor or concept can be used to navigate.

Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
            These may be complex, but their operation should be explained clearly in the comments,
            or the comments should reference an external doc, guide or tutorial.
            Each OpMode should try to only demonstrate a single concept so they are easy to
            locate based on their name.  These OpModes may not produce a drivable robot.

After the prefix, other conventions will apply:

* Sensor class names are constructed as:    Sensor - Company - Type
* Robot class names are constructed as:     Robot - Mode - Action - OpModetype
* Concept class names are constructed as:   Concept - Topic - OpModetype

Each OpMode sample class begins with several lines of code like the ones shown below:

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
 OR
 @Autonomous
```

The name that will appear on the driver station's "opmode list" is defined by the code:
 ``name="Template: Linear OpMode"``
You can change what appears between the quotes to better describe your opmode.
The "group=" portion of the code can be used to help organize your list of OpModes.

As shown, the current OpMode will NOT appear on the driver station's OpMode list because of the
  ``@Disabled`` annotation which has been included.
This line can simply be deleted , or commented out, to make the OpMode visible.

### Importing new modules

In this sample repository the following java libraries have been installed: EasyOpenCV, ZXing and 
FTC dashboard. In case you want to install different or new libraries you can change this code in
the build.gradle (Module :TeamCode) file.
```
dependencies {
    implementation project(':FtcRobotController')
    annotationProcessor files('lib/OpModeAnnotationProcessor.jar')
    implementation 'com.journeyapps:zxing-android-embedded:4.3.0'
    implementation 'org.openftc:easyopencv:1.6.2'
    implementation ('com.acmerobotics.dashboard:dashboard:0.4.9') {
        exclude group: 'org.firstinspires.ftc'
    }
}
```

In case the library has to be downloaded from a different location you can add the location of the
library in 
```
repositories{
    maven { url = 'https://maven.brott.dev/' }
}
```
### Key Modifications to the FTC Repository
The libraries imported above require certain modifications in the FTC repository gradle files. 
In case you are downloading the FtcRobotController from scratch ensure that the following lines are
present in their respective build.gradle files:
In build.gradle (Project: <name>)
```
 dependencies {
        classpath 'com.android.tools.build:gradle:7.4.2'
        classpath "org.jetbrains.kotlin:kotlin-gradle-plugin:1.8.0" 
    }
```
In build.common.gradle:
```
minSdkVersion 24
```