"use strict"; // turn on strict mode
/*jslint browser:true */
/*jslint devel:true*/

/* A set of functions to be used in testing the SIS from a web page.
  The web page should contain blank divs with the following id values:
    1. deviceListOutput
    2. debugLog
    3. functionButtons
    4. variableButtons
    5. commandOutput
    6. debugLogDiv
  To get a page going you should modify one of the existing web pages. Or,
  include this in the header:
    <script src="//cdn.jsdelivr.net/sparkjs/0.2.0/spark.min.js" type="text/javascript"></script>
    <script src="jbsSpark.js" type="text/javascript"></script>
  and then start dealing with JS errors for missing divs and buttons. Every time
  the JS complains of a missing element, just add a DIV with that ID. Eventually
  add this
    <script> SHRIMPWARE.SISClient.setMode("ConfigElmStreet"); </script>
  and that will give you new divs to add to your web page.

  External Calls to Spark.io:
     initWebPage() if there is anything to do
     logAdd(string) will add the string to the top of a time
        stamped list in a div with the id debugLog
     logClear() clears the debugLog div
     loginToSpark() brings up the modal form and starts the
        retrieval of all the attributes of the selected core
     listAllDevices() will display a list of registered devices
        in a div with the id deviceListOutput and populate buttons
        and text boxes in functionButtons and variableButtons
        activeDeviceSet() set the window.activeDevice variable to be
        the selected device from spark.device[ ].
     listAttributes() refreshes the Actions list for the selected core
     callSparkCoreFunctionFromHTMLButton(functionToCall) places a
        call to the spark.io API. The return code from the spark.io
        API is returned.
     getSparkCoreVariableFromHTMLButton(variableToGet) places a call
        to the spark.io API. The value we get back is returned, and placed
        in a global variable in the module.
   External Calls to handle Spark Core communication
     getSensorLog() obtain the most recent sensor events
     getSensorConfig() obtain the currently configured sensors
   Other External Calls
     startMonitoring() in case the persistent event monitor closes
        you can call this from your browser's console. Don't know why the
        connection closes. At some point we'll figure that out and this
        call will remain internal.

A good way to learn this code is to first look at the routines getSparkCoreVariable
and callSparkCoreFunction. These are the basic calls to interact with the cloud.
If you are new to JS you will see that in these routines we make calls like
_activeDevice.call. _activeDevice is a global variable that holds the instance
of the Spark device we are using.

In the comments I try to use the phrase "spark core" when talking about the actual
hardware or cloud. I use the phrase "SIS" when I mean something that has to do with the
firmware we have written to run in the spark core.

(c) 2015 Jim Schrempp
*/

// Module Globals
if (typeof SHRIMPWARE === "undefined") {
  var SHRIMPWARE = {};
} // Start of module declaration
SHRIMPWARE.SISClient = (function() { // private module variables
  var _version = 26,
    //v 26 now expect SIS version 1.00

    _expectedSISCoreVersion = 1.00,  // this Javascript expects this SIS code in the core

    _mainLoop,  // timer that pops every 0.5 seconds, all the time
    _startDate = new Date(), // time this javascript object was created
    _activeDevice,  // object from spark.device() that we want to talk to
    _sparkCoreData = // used to hold the data that comes back from the SIS firmware
        {
            SensorLog: [], // an array of the sensor log after it is read from the SISHub
            SensorLogIsRefreshed: false, // set FALSE when we begin fetching the event log; TRUE when SensorLog is complete
            SensorConfig: [],
            SensorConfigIsRefreshed: false, // set FALSE when we begin fetching the event log; TRUE when SensorConfig is complete
            LastSensorTrip: '',
            SISConfigIsRefeshed: false
            // search code for validateCoreConfig to see what other properties
            // can be part of this object. God Damn, I hate loosly typed languages.
        },
    _lastHeartbeat, // time that we last got a message from the spark cloud.
                                 // used to determine if we have lost the connection
    _attributes, // the attributes of our selected core, returned from the spark cloud

    _lastHandledPublishedEventNum = 0,
    // the last event number that has been handled
    // so we don't handle old published events
    // should probably have one per eventName

    _animationTimer1Sec, // when screen needs to be animated, this pops once a second

    _defaultBtnStyle,

    // One function of the UI is to add a sensor code and description to the
    // configuration array in the SIS. These variables keep track of that activity
    _sensorPositionBeingConfigured = -1,  // the position in the SIS core
                                           // that we will assign the code to.

    _sensorDescriptionBeingConfigured = '', // the description we will set in the SIS

    _eMode = { // the web page sets this so that we know what options to
                 // provide to the user
        DoNothing : {},
        SISDebug : {value: 0, name: "SISDebug", title: "SIS Debug Page",
            sensorList: "", showConfig:true},
        ConfigSmallApartment: {value: 1, name: "ConfigSmallApartment", title: "SIS Small Apartment Setup",
            sensorList: "", showConfig:true },
        ConfigSaratoga: {value: 2, name: "ConfigSaratoga", title: "SIS Saratoga Townhouse Setup",
            sensorList: "", showConfig:true},
        ConfigElmStreet: {value: 3, name: "ConfigElmStreet", title: "SIS Elm Street House Setup",
            sensorList: "", showConfig:true},
        ConfigStandard: {value: 4, name: "ConfigStandard", title: "SIS Client Control",
            sensorList: "", showConfig:true}
    },
    // SIS Firmware expects PIRs in 0-11, Exterior Doors in 12-15, Generic in 16-18, Alarm/Panic in 19.
    _eType = {   // Sensor layout based on the eMode that has been set
        SmallApartment: [
            {pos: 0, label: "FrontRoomPIR", display: "Front Room PIR"},
            {pos: 1, label: "BedRoomPIR",   display: "Bed Room PIR"},
            {pos: 2, label: "BathPIR",      display: "Bath Room PIR"},
            {pos: 3, label: "HallwayPIR",   display: "Hallway PIR"},
            {pos: 12, label: "FrontDoorSep", display: "Front Door Separation"}
        ],
        Saratoga: [
            {pos: 0, label: "FrontRoomPIR",  display: "Front Room PIR"},
            {pos: 1, label: "MasterBedPIR",  display: "Master Bed Room PIR"},
            {pos: 2, label: "SecondBedPIR",  display: "Second Bed Room PIR"},
            {pos: 3, label: "KitchenPIR",    display: "Kitchen PIR"},
            {pos: 4, label: "UpperHallPIR",  display: "Upper Hall PIR"},
            {pos: 12, label: "FrontDoorSep",  display: "Front Door Separation"},
            {pos: 13, label: "GarageDoorSep", display: "Garage Door Separation"}
        ],
        ElmStreet: [
            {pos: 0, label: "FrontRoomPIR",  display: "Front Room PIR"},
            {pos: 1, label: "JimsRoomPIR",   display: "Jim's Office PIR"},
            {pos: 2, label: "KitchenPIR",   display: "Kitchen PIR"},
            {pos: 3, label: "MasterBRPIR",   display: "MasterBR PIR"},
            {pos: 12, label: "FrontDoorSep",  display: "Front Door Separation"},
            {pos: 13, label: "BackDoorSep",  display: "Back Door Separation"},
            {pos: 16, label: "OnePersonHome",  display: "One Person Home"},
            {pos: 17, label: "TwoPeopleHome",  display: "Two People Home"},
            {pos: 18, label: "NoOneHome",  display: "No One Home"},
            {pos: 19, label: "AlertButton", display: "Alert Button"}
        ],
        Standard: [
            {pos: 0, label: "FamilyRoom1PIR",  display: "Family Room 1 PIR"},
            {pos: 1, label: "FamilyRoom2PIR",   display: "Family Room 2 PIR"},
            {pos: 2, label: "KitchenPIR",   display: "Kitchen PIR"},
            {pos: 3, label: "Bedroom1PIR",   display: "Bedroom 1 PIR"},
            {pos: 4, label: "Bedroom2PIR",   display: "Bedroom 2 PIR"},
            {pos: 5, label: "Bedroom3PIR",   display: "Bedroom 3 PIR"},
            {pos: 6, label: "OfficePIR",   display: "Office PIR"},
            {pos: 7, label: "DiningroomPIR",   display: "Dining Room PIR"},
            {pos: 8, label: "Bathroom1PIR",   display: "Bathroom 1 PIR"},
            {pos: 9, label: "Bathroom2PIR",   display: "Bathroom 2 PIR"},
            {pos: 10, label: "Otherroom1PIR",   display: "Other Room 1 PIR"},
            {pos: 11, label: "Otherroom2PIR",   display: "Other Room 2 PIR"},
            {pos: 12, label: "FrontDoorSep",  display: "Front Door Separation"},
            {pos: 13, label: "PatioDoorSep",  display: "Patio Door Separation"},
            {pos: 14, label: "BackDoorSep",  display: "Back Door Separation"},
            {pos: 15, label: "OtherDoorSep",  display: "Other Door Separation"},
            {pos: 16, label: "GenericSensor1",  display: "Generic Sensor 1"},
            {pos: 17, label: "GenericSensor2",  display: "Generic Sensor 2"},
            {pos: 18, label: "GenericSensor3",  display: "Generic Sensor 3"},
            {pos: 19, label: "AlertButton", display: "Alert Button"}
        ]
    },

    _mode = _eMode.SISDebug

    ;
  var
    // called from the web page as soon as it initialises this JS object
    // this essentially tells the rest of the JS what kind of SIS the web
    // page is set up to configure
    setMode = function(modeValue) {
        switch (modeValue) {
        case "":
        case "SISDebug":
            _mode = _eMode.SIS;
            break;
        case "ConfigSmallApartment":
            _mode = _eMode.ConfigSmallApartment;
            _mode.sensorList = _eType.SmallApartment;
            break;
        case "ConfigSaratoga":
            _mode = _eMode.ConfigSaratoga;
            _mode.sensorList = _eType.Saratoga;
            break;
        case "ConfigElmStreet":
            _mode = _eMode.ConfigElmStreet;
            _mode.sensorList = _eType.ElmStreet;
            break;
        case "ConfigStandard":
            _mode = _eMode.ConfigStandard;
            _mode.sensorList = _eType.Standard;
            break;
        default:
            alert ('Your web page must call SHRIMPWARE.SIS.setMode with SIS or ConfigSmallApartment');
            _mode = _eMode.DoNothing;
            break;
        }
    },

    setType = function(modeType) {
        switch (modeType){
        case "Default":
            _mode.type = "";
        }

    },

    // This routine is called by the web page after it has loaded. This is
    // used to initalize all the buttons and other web page elements.
    initWebPage = function() {
        logAdd("Entered initWebPage");
        document.getElementById("jbs_jsversion").innerHTML = "version " + _version;
        document.getElementById("btnListAllDevices").disabled = true;

        _defaultBtnStyle = document.getElementById("btnGetSensorLog").style;
        document.getElementById("configTitle").innerHTML = _mode.title;
        switch (_mode.name) {
        case "":
        case "SISDebug":

            break;
        case "ConfigSmallApartment":
        case "ConfigElmStreet":
        case "ConfigSaratoga":
        case "ConfigStandard":

            document.getElementById("debugLogDiv").style.display = "none";

            break;
        }
        makeSensorTable();

        disableDeviceButtons(true); // do this after the page elements are all set up in the DOM
        disableSensorButtons(true);

        _mainLoop = setInterval(mainLoopTimerPop,500);
        clearLocalSparkConfig();

        //_startDate = new Date();
    },

    // Call this to disable the buttons that should only be shown when a device is selected
    disableDeviceButtons = function(isDisabled) {
        // God, I hate negative logical variable names!
        switch(_mode.name) {
        case "":
        case "SISDebug":
            //document.getElementById("btnGetAttributes").disabled = isDisabled;
            //document.getElementById("btnGetSensorConfig").disabled = isDisabled;
            break;
        case "ConfigSmallApartment":
        case "ConfigElmStreet":
        case "ConfigSaratoga":
        case "ConfigStandard":
            var ele = document.getElementsByName("sensor");
            for(var i=0;i<ele.length;i++) {
                ele[i].checked = isDisabled;
                //ele[i].disabled = isEnabled;
            }
            var visibilityState = "block";
            if (isDisabled) {
                visibilityState = "none";
            }
            document.getElementById("btnClearSISConfig").disabled = isDisabled;
            document.getElementById("btnGetSensorConfig").disabled = isDisabled;

            var tableButtons = document.getElementsByClassName("sensorTableActionButton");
            for (var i=0 ; i<tableButtons.length; i++) {
                tableButtons[i].disabled = isDisabled;
            }

            break;
        }
        document.getElementById("btnGetSensorLog").disabled = isDisabled;
        document.getElementById("btnAnalyzeSensorLog").disabled = isDisabled;
    },

    // Call this to disable the buttons that should only be shown when a sensor is selected
    disableSensorButtons = function(isDisabled) {
        // God, I hate negative logical variable names!
        switch(_mode.name) {
        case "":
        case "SISDebug":
            break;
        case "ConfigSmallApartment":
        case "ConfigElmStreet":
        case "ConfigSaratoga":
        case "ConfigStandard":
            var ele = document.getElementsByName("sensor");
            for(var i=0;i<ele.length;i++) {
                ele[i].checked = isDisabled;
            }
            var visibilityState = "block";
            if (isDisabled) visibilityState = "none";
            document.getElementById("btnSetNewSensorWasTripped").disabled = isDisabled;
            break;
        }
    },

    // The web page has the ability to show a debug log of calls to the cloud.
    // The web page has a button id="debugShowBtn" that calls debugShow to reveal
    // the debug log. The web page has a div id="debugLog" that has the actual
    // log messages. debugLog is wrapped in a div id="debugLogDiv" that controls visibility.
    logAdd = function(message) {
        // adds message to top of the div debugLog with a timestamp
      var currentTime = new Date();
      currentTime = currentTime.getHours() + ":" + currentTime.getMinutes() +
            ":" + currentTime.getSeconds() + "." + currentTime.getMilliseconds();
      currentTime = currentTime.valueOf();
      var logElement = document.getElementById("debugLog");
      var currentLog = logElement.innerHTML;
      currentLog = currentTime + " " + message + "<p>" + currentLog;
      logElement.innerHTML = currentLog;
    },
    logClear = function() {
      document.getElementById("debugLog").innerHTML = "";
    },
    debugShow = function() {
        document.getElementById("debugLogDiv").style.display = "block";
        document.getElementById("debugShowBtn").style.display = "none";
    },

    // Similar to the debug log area, the web page has a place to show command
    // messages. These methods control that area.
    commandOutputAdd = function(message) {
        // adds message to top of the div debugLog with a timestamp
      var logElement = document.getElementById("commandOutput");
      var currentLog = logElement.innerHTML;
      currentLog = message + "<p>" + currentLog;
      logElement.innerHTML = currentLog;
    },
    commandOutputClear = function() {
        document.getElementById("commandOutput").innerHTML = "";
    },

    // The web page has an errorMessages div that displays errors.
    errorMessageAdd = function(message) {
        var logElement = document.getElementById("errorMessages");
        var currentLog = logElement.innerHTML;
        currentLog = message + "<br>" + currentLog;
        logElement.innerHTML = currentLog;
    },

    // If the web page asks this JS to display the current SIS sensor config,
    // this routine is called to add the SIS sensor config to the web page.
    sensorConfigOutputAdd = function(message) {
        // adds message to top of the div debugLog with a timestamp
        var logElement = document.getElementById("commandOutput");
        var currentLog = logElement.innerHTML;
        currentLog = message + "<br>" + currentLog;
        logElement.innerHTML = currentLog;
    },
    sensorConfigOutputClear = function() {
        document.getElementById("commandOutput").innerHTML = "";
    },

    // ------------------------------------------------------------------------
    // MAIN TIMER LOOP
    //
    // This time gets called every 0.5 seconds. It allows this JS to do things
    // without waiting for a user to click a button.

    mainLoopTimerPop = function() {
        // this is called every 0.5 seconds, all the time
        var rightNow = new Date();
        var elapsedMillis = rightNow - _startDate;
        if ( elapsedMillis > 10000) {
            // it's been long enough for the heartbeat to start
            if (rightNow - _lastHeartbeat > 25000) {
                if (_mode.name != "SISDebug") {
                    var elem = document.getElementById("eventHeartbeat");
                    elem.style.background = "#ff0000";
                    var i = 1;
                    // XXX why didn't the line below work?
                    // signalHeartbeatError();
                }
            }
        }
    },
    // ------------ end of main loop ------------

    // ---------- Spark Cloud Login -----------
    // Called by web page to log into Spark Cloud account.
    // Calls listAllDevices when login completes
    loginToSpark = function() {
        // displays a button "Login To Spark" in a div on the page
        // with id="spark-login".  When the user
        // clicks that button and finishes logging in, the function is called.
        // The web page must include this line:
        //<script src="//cdn.jsdelivr.net/sparkjs/0.2.0/spark.min.js" type="text/javascript"></script>
      sparkLogin(sparkLoginComplete);
    },
    sparkLoginComplete = function(data) {
      logAdd("back from sparklogin");
      listAllDevices();
    },
    // -------- end Spark Cloud Login ---------------

    // ------------------ List Devices ------------------
    // Called after Cloud Login completes. Will get a list of all devices
    // registered to that Spark Cloud account. If/when that works, we put a radio
    // for each device on the web page in a div id="deviceListOutput". We
    // then call getAttributes to obtain all the attributes from the cloud for
    // all of the devices registered to the cloud account.
    unselectDevice = function() {
        // when a device is unselected, some of the web page needs to be cleared
        if (_mode.name == "SISDebug") {
            document.getElementById("currentCoreConfig").innerHTML = "select a device";
            document.getElementById("functionButtons").innerHTML = "select a device";
            document.getElementById("variableButtons").innerHTML = "";
        }
    },
    listAllDevices = function() {
        // query spark.io for an updated list of devices
      logAdd("In listAllDevices");
      document.getElementById("btnListAllDevices").disabled = true;
      disableDeviceButtons(true);
      unselectDevice();
      commandOutputClear();
      sensorTableClearSensorCodes();
      var devicesPr = spark.listDevices();
      // NEW TO JS: The next line will execute the spark.listDevices()
      //   method and then call either one of the other of the two routines
      //   below when it has the data from the cloud (or not)
      devicesPr.then(devicesPrTrue, devicesPrFalse);
    },
    devicesPrTrue = function(devices) {
        //we are here when the device information is available
        //so we list information in a div with id="deviceListOutput"
      console.log('Devices: ', devices);
      makeDeviceSelectForm(devices);
      getAttributes();
    },
    devicesPrFalse = function(err) {
      logAdd('List devices call failed: ', err);
      document.getElementById("btnListAllDevices").disabled = false;
    },
    deviceSelectChanged = function() {
        clearLocalSparkConfig();
        var selectList = document.getElementById("deviceSelect");
        if (selectList.selectedIndex === 0)
            return null;
        activeDeviceSet(selectList.options[selectList.selectedIndex].value);

    },


    //------------- Get Attributes ------------------
    // Get all the atttibutes name, id, etc for all devices registered to this
    // spark cloud account.
    getAttributes = function() {
        // Now get all attributes
      logAdd('in getAttributes');
      var attributesPr = spark.getAttributesForAll();
      // NOTE: The above call should do a remote get to the spark.io web site
      // but in the browser console I do not see this happening.
      attributesPr.then(attributesPrTrue, attributesPrFalse);
      document.getElementById("btnListAllDevices").disabled = false;
    },
    attributesPrTrue = function(data) {
        //we are here when the attribute information is available
      logAdd("get attributes succeeded");
      console.log("attributes: ", data);
      _attributes = data;
    },
    attributesPrFalse = function(err) {
      logAdd("get attributes failed");
    },

    // Called by the web page to tell this JS which spark device the web
    // user wants to communicate with. Pass in the "id" of the device as
    // returned from the cloud. At the end, start a persistent connection
    // to the cloud to listen for the core to send spark.Publish events.
    activeDeviceSet = function(idWeWant) {
        // set the global device variable for other functions to use
        // eventually should pop a dialog for user to pick a device
        // for now we just pick device 0
      logAdd("In activeDeviceSet");
      var devList = spark.devices;
      // look throught devList to find the device the user selected
      _activeDevice = null;
      for (var i=0; i < devList.length; i++) {

          if (devList[i].id == idWeWant) {
              _activeDevice = devList[i];
              break;
          }
      }

      if (_activeDevice === null) {
          logAdd("activeDeviceSet error: device not in list");
      } else {
          logAdd("Active device is: " + _activeDevice.name);

          console.log("Active device: " + _activeDevice);
          console.log('Device name: ' + _activeDevice.name);
          console.log('- connected?: ' + _activeDevice.connected);
          if (_activeDevice.connected) {
              listAttributes();
              startMonitoring();    //start listening for events
          } else {
              document.getElementById("deviceListOutput").innerHTML +=
                                "<br>Your Selected Core Is OFFLINE.";
          }
          logAdd("---");

      }

    },

    // Once this JS has been told of the device we should be communicating
    // with, get all the attributes of this device firmware.
    listAttributes = function() {
        //retrive the functions and variables that the core supports
        //and populate buttons for each on the web page
        //console.log("global activedevice: ", _activeDevice);
        //console.log("global attributes: ", attributes);
      var x;
      logAdd("in listAttributes");
      if (_mode == _eMode.SISDebug) {
          disableDeviceButtons(true);
          _attributes.forEach(function(entry) {
              console.log(entry);

              if (entry.id == _activeDevice.id) {
                  // create the buttons to call each available Spark.function
                  var functionButtonsHTML = '';
                  if (entry.functions) {
                      for (x in entry.functions) {
                          var functionName = entry.functions[x];
                          logAdd("Function: " + functionName);
                          functionButtonsHTML += formatCallButton(functionName);
                          //console.log(functionButtons);
                      }
                  } else {
                      functionButtonsHTML = '<p>No Functions reported by cloud';
                  }
                  document.getElementById("functionButtons").innerHTML = functionButtonsHTML;

                  // create the buttons to retrieve each available Spark.variable
                  var variableButtonsHTML = '';
                  if (entry.variables) {
                      for (x in entry.variables) {
                          var variableName = entry.variables[x];
                          logAdd("Variable: " + x + " type: " + variableName);
                          variableButtonsHTML += formatRetrieveButton(x);
                      }
                  } else {
                      variableButtonsHTML = '<p>No Variables reported by cloud';
                  }
                  document.getElementById("variableButtons").innerHTML = variableButtonsHTML;
              }

          });
      }
      disableDeviceButtons(false);
      commandOutputClear();
      sensorConfigOutputClear();
      if(_mode != _eMode.SISDebug) {
          getCoreConfigurationAndSensorConfig();
      }
    },

    // --------------------------------------------------------------------
    // ------- Monitor the spark api for events.
    //
    // This opens a persistent connection to the spark cloud, listening for
    // events. When an event is sent from the cloud, we call processPublishNotificationGroup.
    // New to JS: Note the use of onreadystatechange; we call this to define
    //   what happens when data comes back from the cloud. We call this before
    //   calling the GET function.
    startMonitoring = function() {
        // call only when _activeDevice is defined (after selectActiveDevice)
        logAdd("entered startMonitoring");
        var eventMonitor = new XMLHttpRequest();

        eventMonitor.onreadystatechange = function() {

            signalHeartbeat(); // we got something from the cloud, so the connection
                               // is still open and running

            if (eventMonitor.readyState == 4) {
                errorMessageAdd("Event monitor stream has closed.");
                // xxx here we should try to restart the monitoring again.
            }
            var data = eventMonitor.responseText;
            console.log(data);
            if (data.length > 5) {  // a short length indicates a heartbeat from the cloud
                // go handle the notification from the SIS
                processPublishNotificationGroup(data);
            }
        };

        // listen to only the selected device
        var accessToken = window.spark.accessToken;
        eventMonitor.open("GET", "https://api.spark.io/v1/devices/" +
            _activeDevice.id + "/events/SISEvent?access_token=" +
            accessToken , true);
        eventMonitor.send();
        /*        if(typeof(EventSource) !== "undefined") {
          //var source = new EventSource("demo_sse.php");
                   Spark.getEventStream(false, 'mine', function(data) {
                                 console.log("Event: " + data);            });
        //source.onmessage = function(event) {
        //    addLog(event.data);
        //};
        } else {
               logAdd("Browser does not support server-sent events...");
                  }
        */
    },
    signalHeartbeat = function() {
        // we have received something from the cloud, so the connection is
        // still open and working. We'll flash a little green in a div
        // id="eventHeartbeat" to let the user know things are still alive.
        _lastHeartbeat = new Date();

        switch (_mode.name) {
            case "ConfigSmallApartment":
            case "ConfigElmStreet":
            case "ConfigSaratoga":
            case "ConfigStandard":
                var heartbeat = document.getElementById("eventHeartbeat");
                var hbState = heartbeat.getAttribute("data-state");
                if (hbState == "OFF") {
                    heartbeat.style.background = "#00ff00";
                    heartbeat.setAttribute("data-state", "ON");
                }
                setTimeout(function() {  // used to turn the green off
                    heartbeat.style.background = "#ffffff";
                    heartbeat.setAttribute("data-state", "OFF");
                },500);
                break;
            default:
                // ignore the heartbeat
                break;
        }
    },
    signalHeartbeatError = function() {
        // called by the Mainloop if too long goes by without any message from the cloud
        document.getElementById("eventHeartbeat").style.background = "#ff0000";
    },

    // -------------- process messages from the cloud ---------------
    // Called when we get a message from the cloud that the SIS has published
    // an event. We parse the event and decide what to show the user.
    processPublishNotificationGroup = function(data) {
      // A notice from the cloud can contain several SIS events. Here
      // we sort them out and handle them one event at a time.
      var eventRows = data.split("\n");
      for (var i = 0; i < eventRows.length; i++) {
        if (eventRows[i].substring(0, 4) == "data") {
          var eventName = eventRows[i - 1];
          eventName = eventName.substring(7, eventName.length);
          var eventData = eventRows[i];
          eventData = eventData.substring(6, eventData.length);
          try {
            eventData = JSON.parse(eventData);
            processPublishEvent(eventName, eventData);
          } catch (err) {
            logAdd(err);
            logAdd("Error parsing SparkIO event JSON");
            return;
          }
        }
      }
    },
    processPublishEvent = function(eventName, eventData) {
        // Now we have something to work with! This routine
        // process one event at a time. The events come from the
        // SIS calling spark.publish().
      var sisEvent;
      try {
        //console.log(eventName);
        //console.log(eventData.data);
        sisEvent = JSON.parse(eventData.data);
      } catch (err) {
        logAdd(err);
        logAdd("Error parsing SIS event JSON");
        return;
      }
      //console.log(sisEvent);
      // if the event is newer than the last one we handled,
      // the go get the SIS sensor log.
        if (Number(sisEvent.eventNum) > Number(_lastHandledPublishedEventNum)) {
            _lastHandledPublishedEventNum = sisEvent.eventNum;
            //getSensorLog(); // should eventually call analyze log here.
            switch (_mode.name) {
            case '':
            case 'SISDebug':
                alertSISEventReceived();
                break;
            case 'ConfigSmallApartment':
            case "ConfigElmStreet":
            case "ConfigSaratoga":
            case "ConfigStandard":
/* xxx ANIMATION TO BE REDONE */
                var sensorLocation = Number(sisEvent.sensorLocation);
                sensorTripStartAnimation(sensorLocation);
                break;
            }
        }
    },
    alertSISEventReceived = function() {
      // We get here when an event from the SIS has a sequence number that is
      // larger than the last one we handled here. We set a button to red
      // so the user knows that there are new events to be seen.
      //alert("SIS Sensor Trip Event");
      styleAButton("btnGetSensorLog", 3);
      styleTheAlert(3);

      /*  var popUpDiv = document.createElement("div");
          popUpDiv.innerHTML =        "
          <DIV id='PopUp' style='display: none; position: absolute; left: 100px; top: 50px; "
          +        " border: solid black 1px; padding: 10px; background-color: rgb(200,100,100); "
          +        " text-align: justify; font-size: 12px; width: 135px;' "
          +        " onmouseover="document.getElementById('PopUp').style.display = 'none' ">
              <SPAN id='PopUpText'>TEXT</SPAN>        </DIV>";        document.body.append(popUpDiv);
      */
    },


    // ------------------------------------------------------------------------
    // -------- Retrieve SIS Core Configuration --------
    //
    // This funtions call the SIS to get the current config (number of sensors,
    // circular buffer size, SIS firmware version, etc)
    // The config is stored in _sparkCoreData.config
    // The results are then shown in the div id="currentCoreConfig"
    //
    getCoreConfigurationAndSensorConfig = function() {
        _sparkCoreData.SISConfigIsRefeshed = false;
        getSparkCoreVariable("Config", storeCoreConfigurationAndGetSensorConfig);
    },
    storeCoreConfigurationAndGetSensorConfig = function (data){
        if (data.length < 20) { // I've seen a timeout return "errornull" but don't know if we can count on that
            errorMessageAdd ("Unable to retrieve SIS configuration");
        } else {
            storeCoreConfiguration(data);
            getSensorConfig();
        }
    },
    storeCoreConfiguration = function(data) {
      // store the core configuration

      var configParsed = _sparkCoreData.Config.split(",");
      for (var i = 0; i < configParsed.length; i++) {
        var paramParsed = configParsed[i].split(":");
        _sparkCoreData[paramParsed[0].trim()] = paramParsed[1].trim();
      }
      validateCoreConfig();
      if (_mode.showConfig) {
          displaySparkConfig();
      }
      return;

      function validateCoreConfig() {
        var expectedConfigParams = {
          cBufLen: "num",
          MaxSensors: "num",
          version: "num",
          utcOffset: "num",
          DSTyn: "y/n",
          resetAt: "epoc"
        };

        // Test case below. All params should be errors and one missing
        //var expectedConfigParams = {cBufLen:"y/n", MaxSensors:"y/n", version:"y/n",
        //    utcOffset:"y/n", DST:"num", reset:"y/n", MissingTest:"num", RealErrorCase:"abc"};
        //console.log("Running test code in validateCoreConfig - errors expected");
        var errorMessages = "";
        // fix up resetAT epoc time.
        if (_sparkCoreData.hasOwnProperty("resetAt")) {
          _sparkCoreData.resetAt = _sparkCoreData.resetAt.replace("Z", "");
        }
        for (var paramName in expectedConfigParams) {
          if (!expectedConfigParams.hasOwnProperty(paramName)) continue;
          var paramValue = _sparkCoreData[paramName];
          var expectedParamType = expectedConfigParams[paramName];
          if (!paramName) {
            errorMessages += "<br>Error: did not find config param: " + paramName;
          } else {
            switch (expectedParamType) {
              case "num":
                if (isNaN(paramValue)) {
                  errorMessages += "<br>Error: param " + paramName + ": " +
                            paramValue + " must be a number";
                }
                break;
              case "y/n":
                if (paramValue != 'yes' && paramValue != 'no') {
                  errorMessages += "<br>Error: param " + paramName + ": " +
                            paramValue + " must be yes/no";
                }
                break;
              case "epoc":
                if (isNaN(paramValue)) {
                  errorMessages += "<br>Error: param " + paramName + ": " +
                            paramValue + " must be a number";
                  break;
                }
                if (paramValue < 1420150996) {
                  errorMessages += "<br>Warning: param " + paramName + ": " +
                            paramValue + " seems too small for epoc time.";
                }
                break;
              default:
              // you must have added a parameter type to the array and not updated
              // this switch statement.
                errorMessages += "<br>Error: 1 Javascript problem in validateCoreConfig.";
            }
            _sparkCoreData.SISConfigIsRefeshed = true;
          }
        } // end for
        if (_sparkCoreData.version != _expectedSISCoreVersion) {
          errorMessages += "<br>Warning: Expected SIS firmware version: " +
                            _expectedSISCoreVersion;
        }
        if (errorMessages !== "") {
            errorMessageAdd(errorMessages);
        }
      }
    },

    clearLocalSparkConfig = function() {
        document.getElementById("currentCoreConfig").innerHTML = '';
        _sparkCoreData.SISConfigIsRefeshed = false;
    },

    displaySparkConfig = function() {
        // A human readable summary of the spark core configuration
        var outputDiv = document.getElementById("currentCoreConfig");
        var output='';
        if (!_sparkCoreData.SISConfigIsRefeshed) {
            output = 'No config data. Is a device selected?';
        } else {
            var dateInSISTZ = new Date(Number(_sparkCoreData.resetAt) * 1000);
            var dateInUTC = new Date((Number(_sparkCoreData.resetAt) -
                        Number(_sparkCoreData.utcOffset) * 3600) * 1000);
            output += "SIS firmware version: " + _sparkCoreData.version;
            output += "<br>Last Reset (SIS local time):<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; " + dateInSISTZ.toLocaleString();
            output += "  (" + _sparkCoreData.utcOffset + ")";
            output += "<br>Last Reset (UTC time):<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;" + dateInUTC.toLocaleString();
            output += "<br>SIS locale observes Daylight Savings Time?<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; " + _sparkCoreData.DSTyn;
            output += "<br>Core retains last " + _sparkCoreData.cBufLen + " sensor events.";
            output += "<br>";
        }
        outputDiv.innerHTML += output;
    },


    // ------------------------------------------------------------------------
    // -------- Get current SIS Sensor log -----------------------
    //
    // These routines will talk to the SIS firmware and retrieve the current
    // entries in its circular buffer of sensor data. The log will then
    // be displayed on the web page using the commandOutput() function.
    getSensorLog = function() {

        // iterate through the sensor log on the Spark core and display results
      styleAButton("btnGetSensorLog", 2);
      commandOutputClear();
      commandOutputAdd("Retrieving Sensor Log");
      _sparkCoreData.SensorLogIsRefreshed = false;
      _sparkCoreData.SensorLog = [];
      iterateSensorLog(-1);
    },
    iterateSensorLog = function(buffPosition) {
        // This is called recusively!!!
        // Retrieve the sensor log at buffPosition, then when done call this
        // again with buffPosition-1. Stop when buffPosition is < 0 OR
        // when a retrieved log at buffPosition is null.
        // Start by calling this with the length of the sensor log.
      if (!_sparkCoreData.SISConfigIsRefeshed) {
          console.log('_sparkCoreData is not current in iterateSensorLog');
          return;
      }

      buffPosition = buffPosition + 1;
      if (buffPosition >= _sparkCoreData.cBufLen) {
        logAdd("buffPosition is now greater than bufferLength");
        _sparkCoreData.SensorLogIsRefreshed = true;
        commandOutputAdd("--end of sensor buffer--");
        styleAButton("btnGetSensorLog", 1);
        styleTheAlert(1);
      } else {
        callSparkCoreFunction("ReadBuffer", buffPosition, function(data) {
          if (data < 0) {
            logAdd("error calling ReadBuffer, " + buffPosition);
          } else {
            getSparkCoreVariable("circularBuff", function(data) {

              if (data) {
                var logEntry = massageSensorLog(data);
                var message = '#' + logEntry.sequence + ' ' + logEntry.sensorName +
                     '<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;' + logEntry.epochDateString;
                commandOutputAdd(message);
                //commandOutputAdd(data);
                _sparkCoreData.SensorLog[_sparkCoreData.SensorLog.length] = logEntry;
                iterateSensorLog(buffPosition);

              } else {

                // else data was nil and we've reached the last valid
                // buffer entry
                commandOutputAdd("--Sensor log done--");
                styleAButton("btnGetSensorLog", 1);
                styleTheAlert(1);

              }

            });
          }
        });
      }
    },

    massageSensorLog = function(sensorLogData) {
    // turn the sensor log record into the displayable output
    // sample SIS event log record:
    //     (S:5)Family Room 1 PIR tripped at Mon Nov 9 23:39:42 2015 Z (epoch:1447112382Z)
        if (!_sparkCoreData.SISConfigIsRefeshed) {
            message = '';
            console.log('_sparkCoreData was null in massageSensorLog');
            return 'error';
        }

        var thisLogEntry = {};
        // find the epoc date
        var locationEpoch = sensorLogData.indexOf("epoch:");
        if (locationEpoch > 0) {
            var locationEnd = sensorLogData.lastIndexOf(")");
            var epochTimeString = sensorLogData.substring(locationEpoch+6,locationEnd-1);
            var epochTimeNumber = Number(epochTimeString);

            var epochDate = new Date(0);
            var temp = - epochDate.getTimezoneOffset(); //local browswer gmt offset
            var timezoneDiffMinutes = (_sparkCoreData.utcOffset * 60) - temp; //diff between browser tz and SIS tz

            epochTimeNumber = epochTimeNumber + timezoneDiffMinutes * 60; // this will display the time in SIS timezone
            epochDate.setUTCSeconds(epochTimeNumber);
            var epochDateString = epochDate.toLocaleString();

            //remove date and epoch from end of data
            // xxx wish we didn't have to rely on this kind of thing. Maybe SIS
            // should send a better formatted log message.
            var locationAt = sensorLogData.indexOf(" at ");
            sensorLogData = sensorLogData.substring(0,locationAt);
            var sensorTripSeq = sensorLogData.slice(3,sensorLogData.indexOf(")"));
            var sensorName = sensorLogData.slice(sensorLogData.indexOf(")")+1, sensorLogData.length);

            //message = "At " + epochDateString + ": " + sensorLogData ;
            thisLogEntry.epochDateString = epochDateString;
            thisLogEntry.epochTimeNumber = epochTimeNumber;
            thisLogEntry.sequence = sensorTripSeq;
            thisLogEntry.sensorName = sensorName;
        }
        else
        {
            // message = "could not find epoch date";
            thisLogEntry.sequence = 1;
            thisLogEntry.sensorName = "error parsing sensorlog";
            thisLogEntry.epochTimeNumber = 0;
            thisLogEntry.epochTimeString = "1/1/1900";
        }
        return thisLogEntry;

    },



    // ------- Sensor Configuration ------------------
    // These fuctions will contact the SIS firmware and get a list of what
    // sensors are configured in each of the SIS configuration slots. The
    // results are displayed in the web page by calling sensorConfigOutputAdd()
    //
    // This calls the SIS firmware function Register(read,n) where n is the
    // configuration buffer position to read. It then reads the SIS firmware
    // variable "registration" to get the actual data. It will do this as many
    // times as needed to get all the configured sensor information.
    //
    getSensorConfig = function() {
        // iterate through the sensor log on the SIS and display results
      sensorConfigOutputClear();
      sensorTableClearSensorCodes();
      _sparkCoreData.SensorConfigIsRefreshed = false;
      _sparkCoreData.SensorConfig = [];
      // xxx should really use callwhendone and passbackdata to have a routine
      //     update the global .SensorConfig. With this current implementation
      //     we have to rely on .SensorConfigIsRefreshed to assure validity
      //     of the .SensorConfig
      iterateSensorConfig(-1);
    },
    iterateSensorConfig = function(buffPosition, callWhenDone, passBackData) {
        // This is called recusively!!!
        // Retrieve the sensor config at buffPosition, then when done call this
        // again with buffPosition-1. Stop when buffPosition is < 0.
        // Start by calling this with the length of the sensor config array on
        // the spark core.
        buffPosition++;
        if (buffPosition >= _sparkCoreData.MaxSensors) {
          //here when we are done with recusion
          logAdd("buffPosition is correctly less than 0");
          _sparkCoreData.SensorConfigIsRefreshed = true;
          if (callWhenDone) callWhenDone(passBackData);

        } else {
            sisReadASensorConfig(buffPosition, function(sisConfigItem) {
                _sparkCoreData.SensorConfig[_sparkCoreData.SensorConfig.length] = sisConfigItem;
                iterateSensorConfig(buffPosition, callWhenDone, passBackData);
            });
        }
    },
    sisReadASensorConfig = function(sensorPosition, successFunction) {
        // calls successFunction(data) with an SISConfigItem
        sensorTableUpdateSensorCode({position:sensorPosition ,sensorCode:"--"});
        var commandParam = "read, " + sensorPosition;
        callSparkCoreFunction("Register", commandParam, function(data) {
            if (data < 0) {
                logAdd("error calling registration, " + sensorPosition);
            } else {
                getSparkCoreVariable("registration", function(data) {
                    if (data) {
                        //sensorConfigOutputAdd(data);
                        var sisConfigItem = sisSensorConfigParse(data);
                        sensorTableUpdateSensorCode(sisConfigItem); // always update the table
                        if (successFunction) {
                            successFunction(sisConfigItem);
                        }
                    } else {
                        logAdd("error reading sensor registration in sisReadASensorConfig");
                    }

                });
            }
        });
    },
    sisSensorConfigParse = function (dataFromSIS) {
        //input is the SIS output to a read x of the sensor config
        // e.g.  "loc: 2, sensor code: 34528 is for MasterBRPIR"
        //output is an object with elements for each important part of the sensor config

        var SISConfigItem = {position:"" ,sensorCode:""};
        var inputSplit = dataFromSIS.split(' ');

        var temp = inputSplit[1].trim();
        var commaPos = temp.indexOf(',');
        SISConfigItem.position = temp.substring(0,commaPos);

        SISConfigItem.sensorCode = inputSplit[4].trim();

        return SISConfigItem;

    },

    // ---------- Configure a new sensor in the SIS  ----------------------------
    //
    // These functions are called from the web page in order to guide the user
    // through configuring a new sensor in the SIS firmware. The user starts by
    // selecting a radio button for the sensor they want to configure (it might
    // have a name like "front room"). When they click an Add button then
    // modalsetNewSensorBegin is called to set up the process.
    // modalsetNewSensorBegin is called from a GUI button.
    // setNewSensorWasTripped is called from a GUI once the user has
    //    triggered the new sensor. The sensor is registered in the SIS.
    // Utility functions are also here to clear the entire SIS sensor config and
    // to tell the SIS to save the current sensor config to non volatile storage.
    //

    modalsetNewSensorBegin = function(){
        //read the last sensor trip and put it in a global variable to make
        // sure the next time we read it it will be the new sensor. This is
        // needed to prevent confusion when configuring sensors.
        document.getElementById("modalnewSensorSetupDiv").style.visibility = "visible";
        document.getElementById('commandOutput').innerHTML = '';
        getSparkCoreVariable("sensorTrip", function(data) {

            //if (!data) {
                // had some read error or data from SIS was empty (no recent sensor)

            //} else {
                _sparkCoreData.LastSensorTrip = data;
                document.getElementById('btnSetNewSensorWasTripped').disabled = false;
            //}
        });
    },

    resetSensorRegButtons = function() {
        // utility function. If something goes wrong in configuring, set to start
        // over again.
        document.getElementById('btnSetNewSensorWasTripped').disabled = true;
    },
    setNewSensorWasTripped = function() {
        // read the sensor trip.
        // Be sure it is new.
        // Parse out the new sensor id
        // Then stuff it in the SIS slot we are configuring.
        // Have SIS save config.

        var msgElement = document.getElementById('commandOutput');
        // read the sensor trip
        getSparkCoreVariable("sensorTrip", function(data) {

            if (!data) {
                // had some read error
            } else {
                // be sure it is a new sensor trip
                // TODO: SIS should add a sequence number or date to the sensorTrip so we know if it is a new one
                //if (_sparkCoreData.LastSensorTrip == data) {
                if (false) {
                    // data is same as last time
                    msgElement.innerHTML =
                        "<br>New sensor trip was not detected. Try again.";

                } else {
                    // is it an unregistered sensor?

                    if (data.indexOf("unknown") == -1) {
                        msgElement.innerHTML =
                            "<br>Detected sensor is already configured." +
                            " If this is the correct sensor, then remove it from the config first. Sensor id: " +
                            'xxx';

                    } else {
                        // parse out the new sensor id
                        var sensorIdStart = data.indexOf("code:") + 6;
                        var sensorIdEnd = data.indexOf(" ",sensorIdStart);
                        var sensorId = data.substring(sensorIdStart,sensorIdEnd);

                        // have SIS add it into the configuration
                        var commandParam = 'register,' + _sensorPositionBeingConfigured +
                            ',' + sensorId + ',' + _sensorDescriptionBeingConfigured;

                        callSparkCoreFunction("Register", commandParam, function(data) {
                            if (data != 4) {
                                // some error
                                msgElement.innerHTML =
                                    "<br>Error from SIS Register: " + data;
                            } else {
                                saveSensorConfig();
                                msgElement.innerHTML = '<br>Sensor was successfully registered!';
                                sisReadASensorConfig(_sensorPositionBeingConfigured);
                            }
                        });
                    }
                }
            }
            resetSensorRegButtons();
        });
        document.getElementById("modalnewSensorSetupDiv").style.visibility = "hidden";
    },
    cancelSensorConfig = function() {
        resetSensorRegButtons();
        document.getElementById("modalnewSensorSetupDiv").style.visibility = "hidden";
    },

    hideModalClearSISConfig = function() {
        document.getElementById("modalClearSISConfigDiv").style.visibility = "hidden";
    },

    showModalClearSISConfig = function() {
        document.getElementById("modalClearSISConfigDiv").style.visibility = "visible";
    },

    clearSensorConfig = function() {
        // Call to have the SIS firmware wipe out its sensor config
        hideModalClearSISConfig();

        for (var i=0; i<20; i++) {
            clearASingleSensorConfig(i);
        }

        var msgElement = document.getElementById('commandOutput').innerHTML =
            "<br>Sensor Config was cleared.";

        sensorTableClearSensorCodes();

    },
    clearASingleSensorConfig = function (sensorPosition) {

        function logSensorPositionCleared() {
            logAdd("Sensor Config position cleared" + sensorPosition);
            sisReadASensorConfig(sensorPosition);
            saveSensorConfig();
        }

        var commandParam = "register," + sensorPosition + ",0,unknown";
        logAdd(commandParam);
        callSparkCoreFunction("Register",commandParam, logSensorPositionCleared);

    },

    saveSensorConfig = function() {
        // call to have the SIS firmware store its sensor config in
        // non volatile memory
        var msgElement = document.getElementById('commandOutput');
        var commandParam = "store,1,1,1";
        callSparkCoreFunction("Register", commandParam, function(data) {
            if (data != 4) {
                // some error
                document.getElementById('commandOutput').innerHTML =
                    "Error from SIS Register: " + data;
            } else {
                msgElement.innerHTML = '<br>Sensor setup was successfully saved.';
            }

        });

    },

    // ---------- End of Configure a new sensor in the SIS  ----------------------------

    // ------- Show the user that a sensor was tripped
/* xxx ANIMATION NEEDS TO BE DONE AGAIN */
    sensorTripStartAnimation = function(sensorLocation) {
        // call this to decorate a sensor text on the screen
        // The animation will slowly degrade the color.
        var showSensorActive = document.getElementsByClassName('showSensorActive');
        for (var i=0; i < showSensorActive.length; i++) {

            // using the name property
            var thisSensorActive = showSensorActive[i];
            if (Number(thisSensorActive.attributes.name.value) == sensorLocation) {

                thisSensorActive.style.background = "#00C0FF";
                thisSensorActive.setAttribute("data-animating","YES");

                sensorActiveAnimate();
                break;

            }
        }
    },
    sensorActiveAnimate = function() {
        if (_animationTimer1Sec) return;

        _animationTimer1Sec = setInterval(function() {
            // degrade the decoration of all sensor text
            var needsAnimation = false;
            var showSensorActive = document.getElementsByClassName('showSensorActive');
            for (var i=0; i < showSensorActive.length; i++) {
            //for (var showSensor in showSensorActive) {

                var thisSensorActive = showSensorActive[i];
                var animating = thisSensorActive.getAttribute("data-animating");
                if (animating == "YES") {
                    needsAnimation = true; //not done yet
                    thisSensorActive.style.background = "#ffffff";
                    thisSensorActive.setAttribute("data-animating", "NO");
                    break;

                }
            }
            if (!needsAnimation) {
                clearInterval(_animationTimer1Sec);
                _animationTimer1Sec =  null;
            }

        }, 1000);
    },
// -----------  end of show the user that a sensor was tripped --------




    // --------------------------------------------------------------
    //-------  To call a Spark.function or retrieve a Spark.variable
    //
    getSparkCoreVariable = function(variableToGet, callbackFunction) {
        // This routine calls out to a Spark Core to retrieve a variable
        //should make sure _activeDevice is not null
      _sparkCoreData[variableToGet] = "waiting to get value";
      _activeDevice.getVariable(variableToGet, function(err, data) {
        var returnValue;
        if (err) {
          logAdd("error getting variable" + data); // is this right?
          returnValue = "error" + data;
        } else {
          var message = variableToGet + " current value: " + data.result;
          logAdd(message);
          returnValue = data.result;
        }
        _sparkCoreData[variableToGet] = returnValue;
        callbackFunction(returnValue);
      });
    },
    getSparkCoreVariableFromHTMLButton = function(variableToGet) {
        // call this from a button
      document.getElementById(variableToGet + "Return").value = "waiting";
      var message = getSparkCoreVariable(variableToGet, function(data) {
        document.getElementById(variableToGet + "Return").value = data;
      });
    },
    callSparkCoreFunction = function(functionToCall, stringToSend, callbackFunction) {
        // This routine calls a Spark Core Function and puts the return code from the call
        // in the global array g_htmlValues //should make sure g_activeDevice is not null
      logAdd("Calling Spark Core Function " + functionToCall + " with data: " + stringToSend);
      _sparkCoreData[functionToCall] = "waiting for call to complete";
      _activeDevice.call(functionToCall, stringToSend, function(err, data) {
        var returnValue;
        if (err) {
          logAdd("Error calling function: " + functionToCall + ": " + data);
          // is this right
          returnValue = "error";
        } else {
          logAdd("Back from calling Spark Core Function");
          returnValue = data.return_value;
          console.log("result ", data);
          // seems like return value is always 0...
        }
        _sparkCoreData[functionToCall] = returnValue;
        callbackFunction(returnValue);
      });
    },
    callSparkCoreFunctionFromHTMLButton = function(functionToCall) {
        // This routine pulls the string to send from the HTML document, calls the
        // Spark core function
      var stringToSend = document.getElementById(functionToCall + "Input").value;
      document.getElementById(functionToCall + "Return").value = "waiting";
      var answer = callSparkCoreFunction(functionToCall, stringToSend, function(data) {
        document.getElementById(functionToCall + "Return").value = data;
      });
    },


    // -------------- Utility Functions ------------
    //
    styleAButton = function(btnName, mode) {
        // pass in a button name
        // pass in mode 1,2,3  1:rtn to normal, 2:yellow, 3:red
      var theBtn = document.getElementById(btnName);
      switch (mode) {
        case 1:
          theBtn.style = _defaultBtnStyle;
          break;
        case 2:
          theBtn.style.backgroundColor = '#ffff00';
          break;
        case 3:
          theBtn.style.backgroundColor = '#ff0000';
          break;
        default:
          theBtn.style.backgroundColor = '#000000';
          break;
      }
    },
    styleTheAlert = function(mode) {
        // pass in mode 1,2,3  1:rtn to normal, 3:red
      return; // not using this now
      /*
      var theAlert = document.getElementById("eventReceived");
      switch (mode) {
        case 1:
          theAlert.visibility = 'hidden';
          break;
        case 3:
          theAlert.visibility = 'visible';
          break;
          defaul:
            theAlert.visibility = 'hidden';
          break;
      } */
    },
    formatCallButton = function(sparkFunctionName) {
      /*
      This function returns HTML with one button and two text boxes similar to this:
              <button onclick="callSparkCoreFunctionFromHTMLButton('ReadBuffer')">
              Call ReadBuffer</button>
              &nbsp;&nbsp;
              Input: <input type="text" id="ReadBufferInput" size="15" >
              <br>RtnCode: <input type="text" id="ReadBufferReturn" size="5" >
              <p>
     */
      var sparkFunctionNameData = sparkFunctionName + 'Input';
      var sparkFunctionNameReturn = sparkFunctionName + 'Return';
      var output = '<button onclick="SHRIMPWARE.SISClient.callSparkCoreFunctionFromHTMLButton(' +
        "'" + sparkFunctionName + "')" + '"' + "> Call " + sparkFunctionName +
        '</button>' + "&nbsp;&nbsp;" + 'Input: <input type="text" id="' +
        sparkFunctionNameData + '" size="50" >' + '<br>RtnCode: <input type="text" id="' +
        sparkFunctionNameReturn + '" size="5" ><p>';
      //console.log(output);
      return output;
    },

    formatRetrieveButton = function(sparkVariableName) {
      /*
      This function returns HTML with one button and one text box similar to this:
              <button onclick="variableGet('Buffer_Size')">
              Retrieve Buffer_Size</button>
              <input id="Buffer_SizeReturn" type="text" size="45">
      */
      var sparkVariableNameReturn = sparkVariableName + "Return";
      var output = '<button onclick="SHRIMPWARE.SISClient.getSparkCoreVariableFromHTMLButton(' +
        "'" + sparkVariableName + "'" + ')"> Retrieve ' + sparkVariableName +
        '</button>' + "&nbsp;&nbsp;" + '<input type="text" id="' +
        sparkVariableNameReturn + '" size="60" ><p>';
      return output;
    },

    makeDeviceSelectForm = function(devlist) {
        var outputElement = document.getElementById("deviceListOutput");
        //display all the devices on the web page
        var output = 'Select a device<br><form><select id="deviceSelect">';
        output += '<option disabled selected >-- pick a device --</option>';
        var coreState = "";
        devlist.forEach(function(entry) {
          var connectedValue = "";
          if (entry.connected) {
            coreState = "ONLINE";
          } else {
            coreState = "Offline";
          }
          output += '<option value="' + entry.id + '">' + entry.name + ' ' + coreState + '</option>';
        });
        output += '</select></form>';
        outputElement.innerHTML = output;
        document.getElementById("deviceSelect").onchange = deviceSelectChanged;
    },
/*
    makeSensorLiveDisplay = function() {
        var msg = '';
        for (var i in _mode.sensorList) {
            msg += '<p class="showSensorActive" name="';
            msg += _mode.sensorList[i].pos;
            msg += '">';
            msg += _mode.sensorList[i].display;
            msg += '</p>';
        }
        document.getElementById('sensorActivityDiv').innerHTML = msg;
    },
*/
    // -------------- Sensor Table ------------------
    // This is the code for the main table which displays sensors and
    // allows for them to be changed.
    makeSensorTable = function () {
        //create the main sensor control table
        var theOutput = document.getElementById('sensorTableDiv');
        var content = '';

        //table header
        content += '<table><tr>';
        //content += '<th id="sensorTableColPos">Pos</th>';
        content += '<th id="sensorTableColName">Sensor Name</th>';
        content += '<th id="sensorTableColCode">Current Code</th>';
        content += '<th id="sensorTableColReset">Reset Code</th>';
        content += '<th id="sensorTableColAdd">Add Code</th>';
        content += '</tr>';

        for (var i in _mode.sensorList) {
            var theSensor = _mode.sensorList[i];
            content += '<tr id="sensorTableRow' + theSensor.pos + '">';
        //    content += '<td>' + theSensor.pos + '</td>';
            content += '<td name= "' + theSensor.pos + '" class="showSensorActive">' + theSensor.display + '</td>';
            content += '<td id="sensorTableCellR' + theSensor.pos + 'Code" class="sensorTableCodeCells">' + '--' + '</td>';
            content += '<td class="sensorTableResetCells">' + '<button class="sensorTableActionButton" onclick="SHRIMPWARE.SISClient.sensorTableResetClick(' + theSensor.pos +')">reset</button>' + '</td>';
            content += '<td class="sensorTableAddCells">' + '<button class="sensorTableActionButton" onclick="SHRIMPWARE.SISClient.sensorTableAddClick(' + theSensor.pos +')">add</button>' + '</td>';
            content += '</tr>';
        }

        content += '</table>';

        theOutput.innerHTML = content;

    },
    sensorTableClearSensorCodes = function () {
        var positionCells = document.getElementsByClassName('sensorTableCodeCells');
        for (var i = 0; i < positionCells.length; i++) {
            positionCells[i].innerHTML = '?';
        }
    },
    sensorTableUpdateSensorCode = function (sisConfigItem) {
        // update a sensor code in the displayed table
        var tableCell = "sensorTableCellR" + sisConfigItem.position + "Code";
        document.getElementById(tableCell).innerHTML = sisConfigItem.sensorCode;
    },
    sensorTableAddClick = function (sensorPosition) {
        _sensorPositionBeingConfigured = sensorPosition;
        _mode.sensorList.forEach (function(item){
            if (item.pos == sensorPosition) {
                _sensorDescriptionBeingConfigured = item.display;
                document.getElementById("sensorSetUpName").innerHTML = item.display;
            }
        });
        logAdd('Ready to configure sensor loc: ', sensorPosition);
        modalsetNewSensorBegin();
    },

    sensorTableResetClick = function (sisPosition) {
        clearASingleSensorConfig(sisPosition);

    },

    //---------------  Sensor Table End ----------------------

    analyzeSensorLog = function() {
      commandOutputClear();
      commandOutputAdd("This comes from the routine where I would add analysis.");
      var msg = "Sensor log has " + _sparkCoreData.SensorLog.length + " entries in it.";
      commandOutputAdd(msg);
  };
  return {
      // public methods and properties
    loginToSpark: loginToSpark,
    initWebPage: initWebPage,
    listAllDevices: listAllDevices,
    getSensorLog: getSensorLog,
    getSensorConfig: getSensorConfig,
    clearSensorConfig: clearSensorConfig,
    analyzeSensorLog: analyzeSensorLog,
    logClear: logClear,
    callSparkCoreFunctionFromHTMLButton: callSparkCoreFunctionFromHTMLButton,
    getSparkCoreVariableFromHTMLButton: getSparkCoreVariableFromHTMLButton,
    setMode: setMode,
    setNewSensorWasTripped: setNewSensorWasTripped,
    cancelSensorConfig:cancelSensorConfig,
    debugShow: debugShow,
    saveSensorConfig:saveSensorConfig,
    deviceSelectChanged:deviceSelectChanged,
    hideModalClearSISConfig:hideModalClearSISConfig,
    showModalClearSISConfig:showModalClearSISConfig,
    sensorTableAddClick:sensorTableAddClick,
    sensorTableResetClick:sensorTableResetClick
  };
}());
