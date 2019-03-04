/*Configuratable Cargoship Core by Sunoko*/

//=============BASIC SETTINGS=============//
//flight mode candidate
bool isPassing = true;
bool isAssault = true;
bool isEscape = false;
//flight mode(disable random config when true)
bool isStayAlways = false;
//reinforcement(random)
bool isReinforcement = true;
//reinforcement(always)
bool isReinforcementAlways = true;
//control other ship
bool isCommandShip = true;
//carrying torpedo or drone
bool isCarrier = false;
//boot-up when player approaching
bool isBootUp = false;
//change spawn group when ship in planet garivy well
bool isChangeSpawnGroupInPlanet = true;
//--antenna message config--//
//warning distance scale from initial distance
const double warningDistanceScale = 0.8;
//warning distance upper limit
const double warningDistance = 4000;//m
//warning when player approaching
const string antennaNameWarning = "This is private property. No trespassing.";
//reinforcement distance upper limit
const double reinforceDistanceScale = 0.6;//m
//reinforcement distance upper limit
const double reinforceDistance = 3000;//m
//spawning satellite message
const string spawnerNameDeploySatellite = "Enemy activity detected. Recon probe launching.";
//spawning drone when player approaching
//--space--//
const string spawnerNameSpaceSuit = "Launching assault drones.";
const string spawnerNameSmallShip = "Launching assault drones.";
const string spawnerNameLargeShip = "Launching assault drones.";
//--planet--//
const string spawnerNameSpaceSuitPlanet = "Enemy Scale : Human. Dropper squadron, Attacking position.";
const string spawnerNameSmallShipPlanet = "Enemy Scale : Small Grid. Dropper squadron, Attacking position.";
const string spawnerNameLargeShipPlanet = "Enemy Scale : Large Grid. Dropper squadron, Attacking position.";
//spawner cooltime message
const string antennaNameCooltime = "Recharging communication device";
//spawning escort ship
const string spawnerNameCommandShip1 = "Anomaly detected in radar. Launching recon drone.";//pattern 1
const string spawnerNameCommandShip2 = "No abnormality around.";//pattern 2
//random ship name settings
const string prefix = "";
const string classification = "AOE";
//========================================//

//--------DO NOT EDIT BELLOW CODE---------//
//-------------other settings-------------//
//script name
const string scriptName = "Configuratable Cargoship Core";

const double bootupDistance = 4000;//m
const int spawnerTime = 10;//sec
const double spawnerCooltimeBase = 60;//sec
const double cooltimeDecreaseRate = 50;//percent
const double cooltimeDecreaseSizeLimit = 500;//m
const double weaponsFireRadius = 20;//m
//---------------block list---------------//
List<IMyTerminalBlock> remoteList = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> weaponList = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> thrustList = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> connectBlockList = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> beaconList = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> antennaList = new List<IMyTerminalBlock>();

int blockCount = 0;
//----------------variable----------------//
//--randomized config--//
int navigationMode = (int)NavigationMode.Passing;
bool reinforcement = false;
string spawnerNameCommandShipCurrent = "";
double torpedoLaunchDistance = 4000;//m
//--spawner cooltime--//
string playerType = "none";
double playerRadius = 0;
double spawnerCooltime = 0;
//--other--//
IMyRemoteControl remote;
Vector3D destination = new Vector3D(0,0,0);
Vector3D myPos = new Vector3D(0,0,0);
Vector3D playerPos = new Vector3D(0,0,0);
Vector3D avoidPath = new Vector3D(0,0,0);
Vector3D origin = new Vector3D(0,0,0);
double distance = 0;
double initialAltitude = 0;
float initialSpeedLimit = 17;
double currentWarningDistance = 0;
double currentReinforceDistance = 0;
bool engaged = false;
bool weaponIsWorking = true;
bool consortShipCalled = false;
bool disperse = false;
bool inNaturalGravity = false;
bool spawning = true;
bool breakAway = false;
List<string> fleetIDList = new List<string>();
double currentTimeCalling = 0;
bool avoidAhead = false;
//--run timer--//
const double runPerSec = 0.8;
const double cycle = 1 / runPerSec;
double currentTime = 0;
//--update timer--//
const int updateLimit = 10;
double updateTimer = 0;
string error = "";

public enum NavigationMode
{
    Passing,
    Assault,
    Escape,
    Stay,
}

public Program()//Program() is run once at loading PB
{
    updateTimer = updateLimit + 1;
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
//load config from Custom Data
    if(Me.CustomData.Contains("#end")){
        LoadConfig();
    }
}

public void ArgumentHandler(string argument)
{
    //recieve request join the flight formation
    if(argument.Contains("join")){
        string ID = argument.Replace("join;","");
        if(!fleetIDList.Contains(ID)){
            fleetIDList.Add(ID);
        }
        return;
    }
    if(argument.Contains("TargetType")){
        string[] argumentSpr = argument.Split(';');
        playerType = argumentSpr[0].Replace("TargetType:","");
        Double.TryParse(argumentSpr[1].Replace("Radius:",""),out playerRadius);
        return;
    }
}

public void Main(string argument,UpdateType type)
{
    if(type == UpdateType.Antenna){
        ArgumentHandler(argument);
        return;
    }
    currentTime += Runtime.TimeSinceLastRun.TotalSeconds;
    if (currentTime < cycle){
        return;
    }
    Run();
    currentTime = 0;
}

public void Run()
{
    updateTimer += currentTime;
//get blocks and system failure check
    if(GetBlocks()){
        Echo(error);
        return;
    }
//get player position
    remote = remoteList[0] as IMyRemoteControl;
    foreach(IMyRemoteControl r in remoteList){
        if(r.IsWorking && r.IsFunctional){
            remote = r;
            break;
        }
    }
    myPos = remote.GetPosition();
    if(!remote.GetNearestPlayer(out playerPos)){
        Echo("Player not detected.");
        return;
    }
    distance = Vector3D.Distance(playerPos,myPos);
    //generate random ship name
    if(!Me.CustomData.Contains("#end")){
        RandomConfig();
        foreach(var b in beaconList){
            b.CustomName = RandomNameGenerator();
        }
        currentWarningDistance = distance * warningDistanceScale;
        currentWarningDistance = currentWarningDistance > warningDistance ? warningDistance : currentWarningDistance;
        currentReinforceDistance = distance * reinforceDistanceScale;
        currentReinforceDistance = currentReinforceDistance > reinforceDistance ? reinforceDistance : currentReinforceDistance;
    }
    if(remote.GetNaturalGravity().Length() > 2){//inside natural gravity
        inNaturalGravity = true;
    }
//moving forward when ship stopping(for test spawner)
    if(!isBootUp && !inNaturalGravity){
        SlowMoveForward();
    }
//control other ship
    if(isCommandShip){
        isCommandShip = CommandShipControll();
    }else{
        consortShipCalled = true;
    }
    if(consortShipCalled){
//boot up
        if(isBootUp && distance < bootupDistance){
            BootUp();
            isBootUp = false;
        }
//torpedos/drones launch
        if(isCarrier && distance < torpedoLaunchDistance){
            var pbList = new List<IMyTerminalBlock>();
            GridTerminalSystem.GetBlocksOfType<IMyProgrammableBlock>(pbList);
            foreach(var pb in pbList){
                pb.ApplyAction("OnOff_On");
            }
            foreach(var c in connectBlockList){
                if(c is IMyShipConnector){
                    (c as IMyShipConnector).ApplyAction("Unlock");
                }else if(c is IMyMotorStator){
                    (c as IMyMotorStator).ApplyAction("Detatch");
                }else{
                    c.ApplyAction("OnOff_Off");
                }
            }
            isCarrier = false;
        }
//spawning drones
        spawnerAntennaManager();
//fire weapon
        if(weaponIsWorking){
            WeaponControl();
        }else{
            foreach(IMyUserControllableGun gun in weaponList){
                gun.ApplyAction("Shoot_Off");
            }
        }
//autopilot
        if(distance < reinforceDistance){
            engaged = true;
            if(origin.Length() == 0){
                origin = Me.GetPosition();
            }
        }
        Vector3D gravityVector = remote.GetNaturalGravity();
        if(navigationMode != (int)NavigationMode.Passing && engaged){
            ShipAutoPilot();
        }else if(avoidPath != Vector3D.Zero){
            if(!avoidAhead && GetYawAngle(remote,avoidPath) < 10){
                avoidAhead = true;
            }
            if(avoidAhead){
                avoidPath = remote.WorldMatrix.Forward;
            }
            double currentAltitude = 0;
            remote.TryGetPlanetElevation(MyPlanetElevation.Sealevel, out currentAltitude);
            Vector3D currentDest = avoidPath * 5000 + Vector3D.Normalize(gravityVector) * (currentAltitude - initialAltitude) + myPos;
            AutoPilot(currentDest,true,true);
            if(distance > warningDistance * 2){
                remote.SpeedLimit = 100;
            }else if(Vector3D.Dot(remote.WorldMatrix.Forward,remote.GetShipVelocities().LinearVelocity) < initialSpeedLimit / 2){
                remote.SpeedLimit = 100;
            }else{
                remote.SpeedLimit = initialSpeedLimit;
            }
        }else if(gravityVector != Vector3D.Zero && distance < reinforceDistance + 1000){
            avoidPath = Vector3D.Cross(gravityVector,playerPos - myPos);
            avoidPath = avoidPath * Vector3D.Dot(remote.WorldMatrix.Forward,avoidPath);
            Vector3D _projVector = gravityVector.Dot(playerPos - myPos) / gravityVector.LengthSquared()  * gravityVector;
            Vector3D _reverseDirVector = -(playerPos - myPos) - _projVector;
            avoidPath = Vector3D.Normalize(avoidPath + _reverseDirVector);
            remote.TryGetPlanetElevation(MyPlanetElevation.Sealevel, out initialAltitude);
            initialSpeedLimit = remote.SpeedLimit;
        }
    }
    StoreConfig();
}

private double GetYawAngle(IMyTerminalBlock refernce,Vector3D targetVector)
{
//get forward/up/right vector
    Vector3D forwardVector = refernce.WorldMatrix.Forward;
    Vector3D rightVector = refernce.WorldMatrix.Right;
//projection tmVector to x,y,z plane
    double projX = Vector3D.Dot(targetVector,forwardVector);
    double projZ = Vector3D.Dot(targetVector,rightVector);
//get angle
    return Math.Atan2(projZ,projX);
}

private string RandomNameGenerator()
{
    var NameListSingle = new List<string>{
        "Abigail","Abundance","Acavus","Agamemnon","Agate","Albion","All Aboard","Amethyst","Amity","Aquilon","Antenor","Ares","Artemis","Atlas",
        "Beacon","Beyond","Begonia","Bellona","Bergamot","Binary Star","Blazer","Bounty","Brave New World","Blilliant","Buttercup",
        "Calliope","Calypso","Ceres","Challenger","Cherub","Clover","Comet","Conception","Crocus","Cynthia",
        "Daisy","Delphinium","Deus Ex Machina","Diadem","Diamond","Diligent","Discovery","Druid","Dryad",
        "Echo","Eclipse","Efficiency","Emerald","Endeavour","Endurance","Enterprise","Europa","Event Horizon","Experience","Explorer",
        "Foresight","Forest","Fortune","Forward","Fountain","Freesia","Free Space","Friendship","Frontier",
        "Garden Of Eden","Garnet","Geyser","Gift Of Fortune","Good Fortune","Grace",
        "Harvester","Hecate","Helios","Hesper","Hibiscus","Hilarious",
        "Icarus","Icicle","Indefatigable","Integrity","Intelligence","Intrepid","Invention","Investigator","Iris","Island",
        "Jelly","Jorney","Jump","Jupiter","Jweler",
        "Labor","Luck","Lavender","Lilac",
        "Margaret","Magnitude","Maria","Marigold","Mariner","Martial","Marvel","Mediator","Mercury","Messenger","Mistral",
        "Naiad","Neptune","New Horizon","Nomad","Nymph",
        "Oberon","Ocean","Occasion","Onyx","Opal","Opportunity","Oracle",
        "Paradox","Pegasus","Perseus","Petunia","Pluto","Pulsar","Polaris","Poseidon","President","Prompt","Prophet","Psyche","Pursuer",
        "Quadrant","Qualify","Quality","Quantity","Quarter","Queen","Quick","Quiet",
        "Regulus","Rosemary","Ruby",
        "Sapphire","Searcher","Serenity","Seraph","Snowdrop","Solution","Sprightly","Standard","Stonehenge","Strenuous","Supreme",
        "Topaze","Torrent","Tradewind","Transfer","Transit","Trancend",
        "Ulysses","Undine","Utopia",
        "Vega","Venturer","Venus","Verbena","Verity","Vesper","Voyage",
        "Wage","Wake","Wallet","Wander","Wanderer","Warm","Wave","Wayfarer","Whirlwind",
        "Zealous","Zenith"
    };
    Random r = new System.Random();
    StringBuilder sb = new StringBuilder();
    if(prefix.Length > 0){
        sb.Append(prefix);
        sb.Append(" ");
    }
    sb.Append(NameListSingle[r.Next(0,NameListSingle.Count - 1)]);
    if(r.Next(0,6) < 2){
        sb.Append(ConvertRomanNumerals(r.Next(1,20)));
    }
    sb.Append(" ");
    sb.Append(classification);
    sb.Append("-");
    sb.Append(r.Next(1000).ToString());
    return sb.ToString();
}

private string ConvertRomanNumerals(int arabic)
{
    if(arabic > 3990){
        return "TOO_HIGH";//error
    }
    if(arabic < 1){
        return "TOO_LOW";//error
    }
    var romanCharTable = new Dictionary<string,int>(){
        {"M",1000},
        {"CM",900},
        {"D",500},
        {"CD",400},
        {"C",100},
        {"XC",90},
        {"L",50},
        {"XL",40},
        {"X",10},
        {"IX",9},
        {"V",5},
        {"IV",4},
        {"I",1},
    };
    StringBuilder sb = new StringBuilder(" ");
    int divided = arabic;
    int charCount = 0;
    foreach(var table in romanCharTable){
        charCount = divided / table.Value;
        for(int j = 0;j < charCount;j++){
            sb.Append(table.Key);
        }
        divided -= charCount * table.Value;
    }
    return sb.ToString();
}

private void SlowMoveForward()
{
    remote.DampenersOverride = false;
    float power = 0;
    if(remote.GetShipVelocities().LinearVelocity.Length() < 10 && !engaged){
        power = 100;
    }
    foreach(var th in thrustList){
        string orientation = remote.Orientation.TransformDirectionInverse(th.Orientation.Forward).ToString();
        if(orientation == "Backward"){
            (th as IMyThrust).SetValue<float>("Override",power);
        }
    }
}

private bool CommandShipControll()
{
//calling cosort ship
    if(!consortShipCalled){
        if(currentTimeCalling <= spawnerTime && distance > reinforceDistance){
            AntennaNameChanger(spawnerNameCommandShipCurrent,true);
        }else if(currentTimeCalling >= spawnerTime || distance < reinforceDistance){
            AntennaNameChanger(beaconList[0].CustomName,true);
            currentTimeCalling = 0;
            consortShipCalled = true;
        }
        currentTimeCalling += Runtime.TimeSinceLastRun.TotalSeconds;
    }
//send "disperse" order to consort ships
    if(disperse){
        Transmit("disperse");
        return false;
    }
//assign position for consort ships
    //building message string
    StringBuilder message = new StringBuilder();
    for(int i = 0;i < fleetIDList.Count;i++){
        Vector3D potision = AssignPosition(i);
        message.Append(fleetIDList[i]);
        message.Append("#");
        message.Append(potision.ToString());
        message.Append("|");
    }
    Transmit(message.ToString());
    return true;
}

private void spawnerAntennaManager()
{
//Stage 0 : player is far enough
    if(distance > warningDistance){
        if(isCommandShip){
            AntennaNameChanger(beaconList[0].CustomName,true);
        }else{
            AntennaNameChanger(beaconList[0].CustomName,false);
        }
        currentTimeCalling = 0;
//Stage 1 : notice warning message
    }else if(distance < warningDistance && distance > reinforceDistance){
        AntennaNameChanger(antennaNameWarning,true);
        currentTimeCalling = 0;
//player approach further
    }else if(distance < reinforceDistance){
//Stage 2-1 : disperse consort ship
        if(navigationMode == (int)NavigationMode.Assault){
            disperse = true;
        }
//Stage 2-2 : deploy satellite
        if(reinforcement && playerType == "none"){
            AntennaNameChanger(spawnerNameDeploySatellite,true);
        }
//stage 3 : recieve information from satellite
        if(reinforcement){
            //calculate cooltime
            if(playerRadius > 0 && spawnerCooltime == 0){
                spawnerCooltime = CalculateCoolTime();
            }
            //cooltime manager
            if(playerType != "none"){
                currentTimeCalling += Runtime.TimeSinceLastRun.TotalSeconds;
                if(currentTimeCalling >= spawnerTime + spawnerCooltime){
                    currentTimeCalling = 0;
                }
                if(currentTimeCalling <= spawnerTime){
                    spawning = true;
                }else{
                    spawning = false;
                }
            }
            //spawning drones
            if(spawning){
                if(isChangeSpawnGroupInPlanet && inNaturalGravity){//in planet
                    if(playerType == "CharacterHuman"){
                        AntennaNameChanger(spawnerNameSpaceSuitPlanet,true);
                    }else if(playerType == "SmallGrid"){
                        AntennaNameChanger(spawnerNameSmallShipPlanet,true);
                    }else if(playerType == "LargeGrid"){
                        AntennaNameChanger(spawnerNameLargeShipPlanet,true);
                    }
                }else{//in space
                    if(playerType == "CharacterHuman"){
                        AntennaNameChanger(spawnerNameSpaceSuit,true);
                    }else if(playerType == "SmallGrid"){
                        AntennaNameChanger(spawnerNameSmallShip,true);
                    }else if(playerType == "LargeGrid"){
                        AntennaNameChanger(spawnerNameLargeShip,true);
                    }
                }
            //cooltime
            }else{
                double currentPercentage = Math.Round(((currentTimeCalling  - spawnerTime)/ spawnerCooltime) * 100);
                string msg = antennaNameCooltime + " : " + currentPercentage.ToString() + " %";
                AntennaNameChanger(msg,true);
            }
        }
    }
}

private double CalculateCoolTime()
{
    double decreaseValueBase = spawnerCooltimeBase * (cooltimeDecreaseRate / 100);
    double fixedValue = spawnerCooltimeBase - decreaseValueBase;
    double calculateSize = playerRadius;
    if(calculateSize > cooltimeDecreaseSizeLimit){
        calculateSize = cooltimeDecreaseSizeLimit;
    }
    double decreasedValue = decreaseValueBase * (calculateSize / cooltimeDecreaseSizeLimit);
    return fixedValue + decreasedValue;
}

private Vector3D AssignPosition(int number)
{
    Vector3D leftVector = remote.WorldMatrix.Left;
    Vector3D backwardVector = remote.WorldMatrix.Backward;
    Vector3D myPos = remote.GetPosition();
    Vector3D currentDronePotision;
    int offset = Convert.ToInt32(300 * (1 + (number / 3)));
    int n = number % 3;
    if(n == 2){
        currentDronePotision = (myPos + backwardVector * offset);
        return currentDronePotision;
    }else if(n == 1){
        currentDronePotision = (myPos + leftVector * offset);
        return currentDronePotision;
    }else{
        currentDronePotision = (myPos + (-1 * leftVector * offset));
        return currentDronePotision;
    }
}

private void AntennaNameChanger(string name,bool on)
{
    bool changed = false;
    foreach(var a in antennaList){
    var antenna = a as IMyRadioAntenna;
        if(on){
            if(!changed){
                antenna.CustomName = (name);
                antenna.ApplyAction("OnOff_On");
                if(!antenna.GetValue<bool>("EnableBroadCast")){
                    antenna.ApplyAction("EnableBroadCast");
                }
                changed = true;
            }else{
                antenna.CustomName = ("Antenna");
                antenna.ApplyAction("OnOff_Off");
            }
        }else{
            antenna.CustomName = ("Antenna");
            antenna.ApplyAction("OnOff_Off");
        }
    }
//switch beacon
    if(on){//antennas functional
        foreach(var b in beaconList){
            b.ApplyAction("OnOff_Off");
        }
    }else if(!on || antennaList.Count == 0){
        foreach(var b in beaconList){
            b.ApplyAction("OnOff_On");
        }
    }
}

private void WeaponControl()
{
    Vector3D pmVector = playerPos - myPos;//vector from me to target
    Vector3D forwardVector = remoteList[0].WorldMatrix.Forward * pmVector.Length();//vector from me to straight forward
    double d = Vector3D.Distance(pmVector,forwardVector);
    if(d < weaponsFireRadius){
        foreach(var w in weaponList){
            var weapon = w as IMyUserControllableGun;
            if(weapon is IMySmallGatlingGun){//gatling gun
                if(distance < 850){
                    weapon.ApplyAction("Shoot_On");
                }else{
                    weapon.ApplyAction("Shoot_Off");
                }
            }
            if(weapon is IMySmallMissileLauncher || weapon is IMySmallMissileLauncherReload){//missile
                if(distance < 800){
                    weapon.ApplyAction("Shoot_On");
                }else{
                    weapon.ApplyAction("Shoot_Off");
                }
            }
        }
    }
}

private void ShipAutoPilot()
{
//collision avoidance control
    bool avoidance = true;
    if(Vector3D.Distance(playerPos,myPos) < 1000){//player is reach
        avoidance = false;
    }
//disengagement
    //case 1 : too far from the origin
    if(Vector3D.DistanceSquared(playerPos, origin) > 20000 * 20000){
        navigationMode = (int)NavigationMode.Escape;
    }
    //case 2 : lost all weapons
    if(!weaponIsWorking && (navigationMode == (int)NavigationMode.Assault)){
        Random rnd = new Random();
        if(rnd.Next(0,2) == 0){
            navigationMode = (int)NavigationMode.Escape;
        }
    }
//assault
    if(navigationMode == (int)NavigationMode.Assault){
        destination = playerPos;
//escape
    }else if(navigationMode == (int)NavigationMode.Escape){
        if(breakAway && Vector3D.Distance(destination,myPos) < 100){
            breakAway = false;
        }
        if(!breakAway){
            destination = (myPos - playerPos) * 10000 + myPos;
            breakAway = true;
        }
//stay
    }else if(navigationMode == (int)NavigationMode.Stay){
        AutoPilot(destination,false,avoidance);
        return;
    }
    AutoPilot(destination,true,avoidance);
}

private void AutoPilot(Vector3D dest,bool on,bool collisionAvoidance)
{
    remote.ClearWaypoints();
    string avoidance = "CollisionAvoidance_Off";
    if(collisionAvoidance){
        avoidance = "CollisionAvoidance_On";
    }
    remote.ApplyAction(avoidance);
    remote.AddWaypoint(dest,"Destination");
    remote.SetAutoPilotEnabled(on);
}

private void BootUp()
{
    List<IMyTerminalBlock> blockList = new List<IMyTerminalBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyFunctionalBlock>(blockList);
    foreach(var block in blockList){
        if(!block.IsWorking){
            block.ApplyAction("OnOff_On");
        }
    }
}

private bool CheckWeaponsIsWorking(IMyUserControllableGun gun)
{
    if (!gun.IsFunctional || !gun.IsWorking){
        return false;
    }
    if (gun.HasInventory && !gun.GetInventory(0).IsItemAt(0)){
        return false;
    }
    return true;
}


private void Transmit(string message)
{
    foreach(var a in antennaList){
        var antenna = a as IMyRadioAntenna;
        antenna.ApplyAction("OnOff_On");
        if(!antenna.GetValue<bool>("EnableBroadCast")){
            antenna.ApplyAction("EnableBroadCast");
        }
        antenna.TransmitMessage(message,MyTransmitTarget.Default);
        break;
    }
}

private bool GetBlocks()
{
    Echo(scriptName);
    if(updateTimer < updateLimit){
        Echo("Next Update : " + (Math.Round(updateLimit - updateTimer,1).ToString()));
        if(error.Length > 0){
            return true;
        }
        return false;
    }
    Echo("Blocks Update...");
    updateTimer = 0;
    var blockList = new List<IMyTerminalBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyTerminalBlock>(blockList);
    if(blockList.Count == 0){
        error = "**System Failure**\nBlocks Not Found";
        return true;
    }
    if(blockList.Count == blockCount){
        return false;
    }
//if add or lost blocks
    remoteList.Clear();
    weaponList.Clear();
    thrustList.Clear();
    connectBlockList.Clear();
    beaconList.Clear();
    antennaList.Clear();
//sort blocks
    foreach(var b in blockList){
        if(b is IMyRemoteControl){
            remoteList.Add(b);
        }else if(b is IMyUserControllableGun && CheckWeaponsIsWorking(b as IMyUserControllableGun)){
            weaponList.Add(b);
        }else if(b is IMyThrust){
            thrustList.Add(b);
        }else if(b is IMyShipMergeBlock || b is IMyShipConnector || b is IMyMotorStator){
            connectBlockList.Add(b);
        }else if(b is IMyBeacon){
            beaconList.Add(b);
        }else if(b is IMyRadioAntenna){
            antennaList.Add(b);
        }
    }
//block check
    if(remoteList.Count == 0 && navigationMode != (int)NavigationMode.Passing){//critical failure
        Echo("**System Failure**\nRemote Control Block Not Found");
        return true;
    }
    if(weaponList.Count == 0){//lost all weapons
        weaponIsWorking = false;
    }
    if(antennaList.Count == 0){//lost all antennas
        isCommandShip = false;
    }
    blockCount = blockList.Count;
    return false;
}

private void RandomConfig()
{
    Random r = new System.Random();
//choose navigation mode
    List<int> navigation = new List<int>();
    if(isStayAlways){
        navigationMode = (int)NavigationMode.Stay;
    }else{
        if(isPassing){
            navigation.Add((int)NavigationMode.Passing);
        }
        if(isAssault){
            navigation.Add((int)NavigationMode.Assault);
        }
        if(isEscape){
            navigation.Add((int)NavigationMode.Escape);
        }
        if(navigation.Count == 0){
            navigationMode = (int)NavigationMode.Passing;
        }else{
            navigationMode = navigation[r.Next(0,navigation.Count)];
        }
    }
//choose reinforcement mode
    if(isReinforcementAlways){
        reinforcement = true;
    }else if(isReinforcement){
        if(r.Next(0,2) == 1){
            reinforcement = true;
        }
    }
//choose commandship mode
    if(isCommandShip){
        int cr = r.Next(0,3);
        if(cr == 0){
            isCommandShip = false;//disable commandship mode
        }else if(cr == 1){
            spawnerNameCommandShipCurrent = spawnerNameCommandShip1;
        }else if(cr == 2){
            spawnerNameCommandShipCurrent = spawnerNameCommandShip2;
        }
    }
}

private void StoreConfig()
{
    StringBuilder config = new StringBuilder();

    config.Append(scriptName + "\n");

    config.Append("\n//VARIABLE -- DO NOT EDIT --\n\n");

    config.Append("navigationMode = " + navigationMode.ToString() + "\n");
    config.Append("currentTimeCalling = " + currentTimeCalling.ToString() + "\n");
    config.Append("consortShipCalled = " + consortShipCalled.ToString() + "\n");
    config.Append("spawnerNameCommandShipCurrent = " + spawnerNameCommandShipCurrent + "\n");
    config.Append("reinforcement = " + reinforcement.ToString() + "\n");
    config.Append("currentWarningDistance = " + currentWarningDistance.ToString() + "\n");
    config.Append("currentReinforceDistance = " + currentReinforceDistance.ToString() + "\n");
    config.Append("playerType = " + playerType + "\n");
    config.Append("playerRadius = " + playerRadius + "\n");
    config.Append("origin = " + origin.ToString() + "\n");
    config.Append("avoidPath = " + avoidPath.ToString() + "\n");

    config.Append("fleetIDList = $");
    if(fleetIDList.Count > 0){
        foreach(var t in fleetIDList){
            config.Append(t + "$");
        }
        config.Append(";");
    }else{
        config.Append("none;");
    }

    config.Append("\n#end");

    Me.CustomData = config.ToString();
}

private void LoadConfig()
{
    string[] configSplit = Me.CustomData.Split(new char[] { '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries);
    foreach(var str in configSplit){
        if(str.Contains("//") || str.Contains("!none") ||str.Contains("#end")){
            continue;
        }
        if(str.Contains("fleetIDList")){
            fleetIDList.Clear();
            string[] idStr = str.Split('$');
            for(int i = 0; i < idStr.Length; i++){
                if(idStr[i].Contains("fleetIDList = ")){
                    continue;
                }
                if(idStr[i].Length > 0){
                    fleetIDList.Add(idStr[i]);
                }
            }
            continue;
        }

        if(ConvertVariable(str,"navigationMode = ",ref navigationMode)){continue;}
        if(ConvertVariable(str,"currentTimeCalling = ",ref currentTimeCalling)){continue;}
        if(ConvertVariable(str,"consortShipCalled = ",ref consortShipCalled)){continue;}
        if(ConvertVariable(str,"spawnerNameCommandShipCurrent = ",ref spawnerNameCommandShipCurrent)){continue;}
        if(ConvertVariable(str,"reinforcement = ",ref reinforcement)){continue;}
        if(ConvertVariable(str,"playerType = ",ref playerType)){continue;}
        if(ConvertVariable(str,"playerRadius = ",ref playerRadius)){continue;}
        if(ConvertVariable(str,"origin = ",ref origin)){continue;}
        if(ConvertVariable(str,"avoidPath = ",ref avoidPath)){continue;}
    }
}

public bool ConvertVariable(string line,string name,ref bool variable)
{
    if(line.Contains(name)){
        Boolean.TryParse(line.Replace(name,""),out variable);
        return true;
    }
    return false;
}

public bool ConvertVariable(string line,string name,ref int variable)
{
    if(line.Contains(name)){
        Int32.TryParse(line.Replace(name,""),out variable);
        return true;
    }
    return false;
}

public bool ConvertVariable(string line,string name,ref float variable)
{
    if(line.Contains(name)){
        Single.TryParse(line.Replace(name,""),out variable);
        return true;
    }
    return false;
}

public bool ConvertVariable(string line,string name,ref double variable)
{
    if(line.Contains(name)){
        Double.TryParse(line.Replace(name,""),out variable);
        return true;
    }
    return false;
}

public bool ConvertVariable(string line,string name,ref string variable)
{
    if(line.Contains(name)){
        variable = line.Replace(name,"");
        return true;
    }
    return false;
}

public bool ConvertVariable(string line,string name,ref Vector3D variable)
{
    if(line.Contains(name)){
        Vector3D.TryParse(line.Replace(name,""),out variable);
        return true;
    }
    return false;
}