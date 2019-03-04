/*NPC Plane Autopilot by Sunoko*/

//=============BASIC SETTINGS=============//
//Remote name
const string remoteName = "Remote Control";
//Rotor name({rotorName}1; {rotorName}2; ...)
const string rotorName = "DropperRotor";
//Drop trigger timer name
const string timerName = "TriggerTimer";
//Enable random config
bool randomConfig = true;
//Start by command
bool startByCommand = false;
//Activate command
const string commandActive = "Active";
//Cargo drop distance
double dropDistance = 1200;
//Cargo drop interval
double dropInterval = 1;
//Cruise speed
float cruiseSpeed = 65;
//cruis control speed margin(m/s)
float speedMargin = 0.1f;
//Parachute auto deploy height
float deployHeight = 250;
//Flight mode
FlightMode flightMode = FlightMode.Wandering;
//----==Weapon Settings==----//
//Weapon range
double weaponRange = 800;
//Weapon muzzle velocity
double muzzleVelocity = 400;
//Weapon muzzle velocity cap
double maxMuzzleVelocity = 0;
//Projectile Acceration
double projAcceration = 0;
//Enable seqenced fire
bool sequencedFire = true;
//Sequenced weapon name
const string sequencedName = "sequenced";
//Fire interval
double fireCycle = 1;
//Enable restrict the number of fixed weapon
bool restrictFixedWeaponCount = true;
//Max fixed weapon count
int maxFixedWeaponCount = 6;
//Enable restrict the number of turret
bool restrictTurretCount = true;
//Max turret count
int maxTurretCount = 6;
//Apply fire when target angle within this range
double applyFireAngle = 10;
//----==Flight Settings==----//
//---Cruise Mode---//
//Minimum elevation
double cruiseMinimumElevation = 100;
//---Counter-Air Mode---//
//Elevation threshold at which counter air mode activating
double couterairMinimumElevation = 300;
//Minimum distance from target
int couterairOffset = 500;
//----Wandering Mode---//
//Area radius
int wanderingArearadius = 150;
//Elevation
double wanderingElevation = 350;
//----Dive and Zoom Mode---//
//Away distance
double dnzDistance = 1500;
//Elevation
double dnzElevation = 500;
//----Orbit Mode---//
//Speed
double orbitSpeed = 45;
//Distance offset
double orbitDistance = 550;
//Elevation
double orbitElevation = 300;
//----Hold Mode---//
//Distane
double holdDistance = 550;
//Elevation
double holdElevation = 200;
//========================================//

//--------DO NOT EDIT BELLOW CODE---------//
//-------------other settings-------------//
//script name
const string scriptName = "NPC Plane Autopilot";
//PID Gain
float Kp = 0.02f;
float Ki = 0f;
float Kd = 0.001f;
//integral decrease factor
float Decay = 0.1f;
//adding differencial threshold
float DifferencialThreshold = 0.52f;//nearly 30 degree
//Target leading bias PID Gain
float LeadBias_Kp = 0.002f;
float LeadBias_Ki = 0f;
float LeadBias_Kd = 0.001f;
//Integral decay factor
float LeadBias_Decay = 0.01f;
//Adding differencial threshold
float LeadBias_DifferencialThreshold = 0;

const float RadToDegF = (float)(180 / Math.PI);
//---------------block list---------------//
List<IMyTerminalBlock> remoteList = new List<IMyTerminalBlock>();
List<ThrustAxis> thrustAxisList = new List<ThrustAxis>();
List<IMyTerminalBlock> gyroList = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> gasSupplyList = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> weaponList = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> sequencedWeaponList = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> turretList = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> rotorList = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> timerList = new List<IMyTerminalBlock>();

//int blockCount = 0;
//----------------variable----------------//
MyIni ini = new MyIni();
IMyRemoteControl remote;
Random r = new System.Random();
PIDControl YawPID;
PIDControl PitchPID;
PIDControl RollPID;
PIDControl LeadBiasPID;
IEnumerator<bool> dropStateMachine;
IEnumerator<bool> counterairStateMachine;
IEnumerator<bool> wanderingStateMachine;
IEnumerator<bool> diveAndZoomStateMachine;
Vector3D destinationDir = Vector3D.Zero;
Vector3D aimingDir = Vector3D.Zero;
Vector3D inputLocal = Vector3D.Zero;
Vector3D velocityLocal = Vector3D.Zero;
Vector3D gravityLocal = Vector3D.Zero;
Vector3D destLocal = Vector3D.Zero;
Vector3D gravity = Vector3D.Zero;
Vector3D targetPos = Vector3D.Zero;
Vector3D prevTargetPos = Vector3D.Zero;
Vector3D prevPredictedPos = Vector3D.Zero;
Vector3D targetVelocity = Vector3D.Zero;
Vector3D myPos = Vector3D.Zero;
Vector3D myPrevPos = Vector3D.Zero;
Vector3D myVelocity = Vector3D.Zero;
bool active = false;
bool start = false;
bool isInPlanet = false;
bool horizontalFlight = true;
int moveDir = 0;
double targetSpeed = 0;
double fireInterval = 0;
int sequenceStep = 0;
double fireDelay = 0;
bool targetLeading = true;
double _PrevT = 2;
double bias = 0;

public enum FlightMode
{
    Wandering,
    DiveAndZoom,
    Orbit,
    Hold,
}

//--run timer--//
const double runPerSec = 6;
const double cycle = 1 / runPerSec;
double currentTime = 0;
//--update timer--//
const int updateLimit = 10;
double updateTimer = 0;
string error = "";

public Program()//Program() is run once at loading PB
{
    Runtime.UpdateFrequency = UpdateFrequency.Update1;
    updateTimer = updateLimit + 1;
    ConfigHandler(true);
    if(randomConfig){
        flightMode = GetRandomFlightMode(r);
    }
}

private FlightMode GetRandomFlightMode(Random r)
{
    int random = r.Next(0,4);
    switch(random)
    {
        case 0:
            return FlightMode.Wandering;
        case 1:
            return FlightMode.DiveAndZoom;
        case 2:
            return FlightMode.Orbit;
        case 3:
            return FlightMode.Hold;
        default:
            return FlightMode.Orbit;
    }
}

public void ArgumentHandler(string argument)
{
    if(startByCommand && argument == commandActive){
        active = true;
        return;
    }
}

public void Main(string argument,UpdateType type)
{

    if(type != UpdateType.Update1 && type != UpdateType.Update10 && type != UpdateType.Update100 && type != UpdateType.Once){
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
//Get blocks and system failure check
    if(GetBlocks()){
        Echo(error);
        return;
    }
    if(startByCommand && !active){
        Echo("Active Command = False");
        return;
    }
    foreach(IMyRemoteControl r in remoteList){
        if(r == null){
            continue;
        }
        remote = r;
        break;
    }
    if(remote == null){
        Echo("All RC Blocks are destroyed.\nSystem shutting down.");
        return;
    }
    fireInterval = fireCycle / sequencedWeaponList.Count;
    	IMyProgrammableBlock primaryPB = GridTerminalSystem.GetBlockWithName("Programmable block") as IMyProgrammableBlock;
	if(primaryPB != null && primaryPB.IsFunctional) {
	        Echo("Primary PB active.");
		return;
	}
	else {
		List<IMyRadioAntenna> antennae = new List<IMyRadioAntenna>();
		GridTerminalSystem.GetBlocksOfType(antennae);
		if(antennae.Any()) {
			IMyRadioAntenna antenna = antennae[0];
			antenna.ApplyAction("OnOff_On");
			if(!antenna.GetValue<bool>("EnableBroadCast")) {
				antenna.ApplyAction("EnableBroadCast");
			}
		}
	}
    //-------------Select Target Section-------------//
    myPos = (remote as IMyShipController).CenterOfMass;
    if(!remote.GetNearestPlayer(out targetPos)){
        Echo("Player not detected.");
        return;
    }
    start = true;
    gravity = remote.GetNaturalGravity();
    if(gravity == Vector3D.Zero){
        Echo("Natural Gravity not detected.");
        return;
    }
//-------------Information Section-------------//
    targetSpeed = cruiseSpeed;
    double distanceSquared = (targetPos - myPos).LengthSquared();

    if(prevTargetPos != Vector3D.Zero){
        targetVelocity = (targetPos - prevTargetPos) / currentTime;
    }else{
        targetVelocity = Vector3D.Zero;
    }
    if(targetVelocity.LengthSquared() > 300 * 300){
        targetVelocity = Vector3D.Zero;
    }
    prevTargetPos = targetPos;
    if(myPrevPos != Vector3D.Zero){
        myVelocity = (myPos - myPrevPos) / currentTime;
    }
    myPrevPos = myPos;
//-------------Cargo Drop Section-------------//
    if(rotorList.Any() && distanceSquared < dropDistance * dropDistance && dropStateMachine == null){
        dropStateMachine = DropSequence(currentTime,dropInterval).GetEnumerator();
    }
    if(dropStateMachine != null && !dropStateMachine.MoveNext()){
        dropStateMachine.Dispose();
        dropStateMachine = null;
    }
    if(!rotorList.Any()){
        horizontalFlight = false;
    }
//-------------Flight Path Section-------------//
    double myElevation;remote.TryGetPlanetElevation(MyPlanetElevation.Surface, out myElevation);
    double elevationDifferenceFromTarget = Vector3D.Normalize(-gravity).Dot(targetPos - myPos);
    double targetElevation = myElevation + elevationDifferenceFromTarget;
    GetDestination(distanceSquared,myElevation,targetElevation);
    if(targetLeading)aimingDir = InterceptPoint(remote,remote.WorldMatrix.Forward,aimingDir + myPos,myPos,targetVelocity,myVelocity,muzzleVelocity,maxMuzzleVelocity,projAcceration,currentTime);
//-------------Autopilot Section-------------//
//Thrust control
    bool hydrogenAvailable = CheckHydrogen();
    destLocal = Vector3D.TransformNormal(destinationDir,MatrixD.Transpose(remote.WorldMatrix));
    inputLocal = GetThrustInput(Vector3D.TransformNormal(destinationDir,MatrixD.Transpose(remote.WorldMatrix)),cruiseSpeed);
    velocityLocal = Vector3D.TransformNormal(remote.GetShipVelocities().LinearVelocity,MatrixD.Transpose(remote.WorldMatrix));
    gravityLocal =  gravity != Vector3D.Zero ? Vector3D.TransformNormal(gravity,MatrixD.Transpose(remote.WorldMatrix)) : Vector3D.Zero;
    float shipMass = remote.CalculateShipMass().PhysicalMass;
    foreach(ThrustAxis axis in thrustAxisList){
        axis.Run(destLocal,inputLocal,velocityLocal,gravityLocal,shipMass,hydrogenAvailable,speedMargin,(float)currentTime);
    }
//Gyro control
    Vector2 rotation = GetXYAngle(remote,aimingDir,myPos);
    Vector2 rotationGravity = GetGravityAlingment(remote,gravity);
    float pitchAngle = horizontalFlight ? rotationGravity.X : rotation.X;
    float _pitchInput = PitchPID.Run(pitchAngle * RadToDegF,(float)currentTime);
    float _yawInput = YawPID.Run(rotation.Y * RadToDegF,(float)currentTime);
    float _rollInput = RollPID.Run(-rotationGravity.Y * RadToDegF,(float)currentTime);
    GyroControl(remote,_yawInput,_pitchInput,_rollInput,true);
//-------------Weapon Controll Section-------------//
    if(!rotorList.Any()){
        WeaponControl(distanceSquared,targetPos,myPos);
    }
    if(restrictTurretCount){
        TurretCountControl();
    }
}

private IEnumerable<bool> DropSequence(double currentTime,double timeLimit)
{
    double time = 0;
    while(rotorList.Any()){
        var top = rotorList[0] as IMyMechanicalConnectionBlock;
        foreach(IMyTimerBlock timer in timerList){
            if(timer.CubeGrid != top.TopGrid){
                continue;
            }
            timer.ApplyAction("TriggerNow");
            (rotorList[0] as IMyMotorStator).ApplyAction("Detach");
            rotorList[0].CustomData = "Detached";
            rotorList.Remove(rotorList[0]);
            break;
        }
        while(timeLimit > time){
            time += currentTime;
            yield return true;
        }
        time = 0;
    }
    yield return true;
}

private void GetDestination(double distanceSq,double myElevation,double targetElevation)
{
    isInPlanet = (myElevation != 0 || targetElevation != 0);
//Dive and Zoom Sequence
    if(diveAndZoomStateMachine != null){
        if(!diveAndZoomStateMachine.MoveNext()){
            diveAndZoomStateMachine.Dispose();
            diveAndZoomStateMachine = null;
        }
        return;
    }

//Pre-Attack cruising
    if(distanceSq > 1000 * 1000){
    //Dispose all flight state machine
        if(counterairStateMachine != null){
            counterairStateMachine.Dispose();
            counterairStateMachine = null;
        }
        if(wanderingStateMachine != null){
            wanderingStateMachine.Dispose();
            wanderingStateMachine = null;
        }
        //Emergency crash avoidance
        if(isInPlanet && myElevation < cruiseMinimumElevation){
            Vector3D tmVector = targetPos - myPos;
            Vector3D projection = Vector3D.Dot(tmVector,gravity) / gravity.LengthSquared()  * gravity;
            aimingDir = tmVector - projection;
            destinationDir = (-Vector3D.Normalize(gravity) * 300) + (Vector3D.Normalize(aimingDir) * 100);
            return;
        }
    //Go to target position
        if(flightMode == FlightMode.Hold){
            Vector3D elevationAdjust = Vector3D.Zero,horizontalDestination = Vector3D.Zero;
            if(isInPlanet){
                elevationAdjust = Vector3D.Normalize(-gravity) * (targetElevation < holdElevation ? (holdElevation - myElevation) : targetElevation);
                Vector3D dest = targetPos - myPos + Vector3D.Normalize(myPos - targetPos) * holdDistance;
                horizontalDestination = dest - Vector3D.Dot(dest,gravity) / gravity.LengthSquared()  * gravity;
            }
            destinationDir = horizontalDestination + elevationAdjust;
            aimingDir = targetPos - myPos;
        }else{
            Vector3D elevationAdjust = Vector3D.Zero;
            if(isInPlanet){
                elevationAdjust = Vector3D.Normalize(-gravity) * Math.Max(0,0.7 * cruiseMinimumElevation - targetElevation);
            }
            destinationDir = targetPos - myPos + elevationAdjust;
            aimingDir = destinationDir;
        }
        return;
    }
//Counter-Air mode
    if(isInPlanet && targetElevation > couterairMinimumElevation){
    //Dispose all flight state machine
        if(diveAndZoomStateMachine != null){
            diveAndZoomStateMachine.Dispose();
            diveAndZoomStateMachine = null;
        }
        if(wanderingStateMachine != null){
            wanderingStateMachine.Dispose();
            wanderingStateMachine = null;
        }
        aimingDir = targetPos - myPos;
        if(counterairStateMachine != null){
            if(!counterairStateMachine.MoveNext()){
                counterairStateMachine.Dispose();
                counterairStateMachine = CounterAirSequence().GetEnumerator();
            }
        }else{
            counterairStateMachine = CounterAirSequence().GetEnumerator();
        }
        return;
    }
//Wandering mode
    if(flightMode == FlightMode.Wandering){
        targetLeading = false;
        horizontalFlight = true;
        if(wanderingStateMachine != null){
            if(!wanderingStateMachine.MoveNext()){
                wanderingStateMachine.Dispose();
                wanderingStateMachine = WanderingSequence(isInPlanet,targetElevation).GetEnumerator();
            }
        }else{
            wanderingStateMachine = WanderingSequence(isInPlanet,targetElevation).GetEnumerator();
        }
        aimingDir = destinationDir;
        return;
    }
//Dive and Zoom mode
    if(flightMode == FlightMode.DiveAndZoom){
        if(diveAndZoomStateMachine == null){
            diveAndZoomStateMachine = DiveAndZoomSequence(isInPlanet,targetElevation).GetEnumerator();
        }
        return;
    }
//Orbit mode
    if(flightMode == FlightMode.Orbit){
        aimingDir = targetPos - myPos;
        if(moveDir == 0){
            moveDir = r.Next(1,3);
        }
        Vector3D dir = Vector3D.Zero;
        if(moveDir == 1){
            dir = remote.WorldMatrix.Left;
        }else if(moveDir == 2){
            dir = remote.WorldMatrix.Right;
        }
        targetSpeed = orbitSpeed;
        Vector3D adjust = Vector3D.Zero;
        if(isInPlanet){
            Vector3D elevationAdjust = targetElevation < orbitElevation ? Vector3D.Normalize(-gravity) * (orbitElevation - myElevation) : Vector3D.Normalize(-gravity) * (targetElevation);
            Vector3D distanceAdjust = Vector3D.Normalize(Vector3D.Cross(remote.WorldMatrix.Left,gravity)) * (orbitDistance - Math.Sqrt(distanceSq));
            adjust = elevationAdjust + distanceAdjust;
        }else{
            adjust = remote.WorldMatrix.Forward * (orbitDistance - Math.Sqrt(distanceSq));
        }
        destinationDir = (dir * 1000) + adjust;
        return;
    }
//Hold mode
    if(flightMode == FlightMode.Hold){
        if(isInPlanet){
            Vector3D elevationAdjust = Vector3D.Normalize(-gravity) * (targetElevation < holdElevation ? (holdElevation - myElevation) : targetElevation);
            Vector3D dest = targetPos - myPos + Vector3D.Normalize(myPos - targetPos) * holdDistance;
            Vector3D horizontalDestination = dest - Vector3D.Dot(dest,gravity) / gravity.LengthSquared()  * gravity;
            destinationDir = horizontalDestination + elevationAdjust;
        }else{
            destinationDir = targetPos - myPos + Vector3D.Normalize(myPos - targetPos) * holdDistance;
        }
        aimingDir = targetPos - myPos;
        return;
    }
}

private IEnumerable<bool> CounterAirSequence()
{
    Vector3D randomArea = new Vector3D(r.Next(-couterairOffset,couterairOffset),r.Next(-couterairOffset,couterairOffset),r.Next(-couterairOffset,couterairOffset));
    Vector3D projectionGrav = Vector3D.Dot(gravity,randomArea) / gravity.LengthSquared() * gravity;
    randomArea = randomArea - projectionGrav;
    destinationDir = targetPos - myPos + randomArea;
    yield return true;
    while(destinationDir.LengthSquared() > 50 * 50){
        destinationDir = targetPos - myPos + randomArea;
        yield return true;
    }
    yield return true;
}

private IEnumerable<bool> WanderingSequence(bool isInPlanet,double targetElevation)
{
    Vector3D randomArea = new Vector3D(r.Next(-wanderingArearadius,wanderingArearadius),r.Next(-wanderingArearadius,wanderingArearadius),r.Next(-wanderingArearadius,wanderingArearadius));
    if(isInPlanet){
        Vector3D projectionGrav = Vector3D.Dot(gravity,randomArea) / gravity.LengthSquared() * gravity;
        Vector3D elevationAdjust = targetElevation < wanderingElevation ? Vector3D.Normalize(-gravity) * (wanderingElevation - targetElevation) : Vector3D.Zero;
        randomArea = (randomArea - projectionGrav) + elevationAdjust;
    }
    destinationDir = targetPos - myPos + randomArea;
    yield return true;
    while(destinationDir.LengthSquared() > 50 * 50){
        destinationDir = targetPos - myPos + randomArea;
        yield return true;
    }
    yield return true;
}

private IEnumerable<bool> DiveAndZoomSequence(bool isInPlanet,double targetElevation)
{
    destinationDir = (targetPos - myPos) * 10;
    aimingDir = targetPos - myPos;
    yield return true;
    while((targetPos - myPos).LengthSquared() > 500 * 500){
        destinationDir = (targetPos - myPos) * 10;
        aimingDir = targetPos - myPos;
        yield return true;
    }
    moveDir = r.Next(1,3);
    Vector3D dir = Vector3D.Zero;
    if(moveDir == 1){
        dir = remote.WorldMatrix.Left;
    }else if(moveDir == 2){
        dir = remote.WorldMatrix.Right;
    }
    Vector3D adjust = Vector3D.Zero;
    if(isInPlanet){
        Vector3D elevationAdjust = targetElevation < dnzElevation ? Vector3D.Normalize(-gravity) * (dnzElevation - targetElevation) : Vector3D.Zero;
        Vector3D awayForward = Vector3D.Normalize(Vector3D.Cross(remote.WorldMatrix.Right,gravity)) * dnzDistance;
        adjust = elevationAdjust + awayForward;
    }else{
        adjust = remote.WorldMatrix.Forward * dnzDistance;
    }
    Vector3D awayDir = dir * dnzDistance / 2;
    Vector3D dnzAwayDestination = targetPos - myPos + adjust + awayDir;
    destinationDir = dnzAwayDestination;
    aimingDir = targetPos - myPos;
    yield return true;
    while((targetPos - myPos).LengthSquared() < dnzDistance * dnzDistance){
        destinationDir = dnzAwayDestination;
        aimingDir = targetPos - myPos;
        yield return true;
    }
    yield return true;
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

private void WeaponControl(double distanceSq,Vector3D playerPos,Vector3D myPos)
{
    Vector3D pmVector = playerPos - myPos;//vector from me to target
    Vector3D forwardVector = remote.WorldMatrix.Forward * pmVector.Length();//vector from me to straight forward
    double angle = (Math.Acos(Vector3D.Dot(forwardVector,pmVector) / (forwardVector.Length() * pmVector.Length()))) * (180/Math.PI);
    bool applyFire = false;
    if(angle < applyFireAngle){
        applyFire = true;
    }
    if(distanceSq < weaponRange * weaponRange && applyFire){
        FireWeapon();
    }else{
        StopWeapon();
    }
}

private void TurretCountControl()
{
    int count = 0;
    foreach(IMyLargeTurretBase t in turretList){
        if(count >= maxTurretCount){
            t.SetValue<float>("Range",1);
            continue;
        }
        if(!CheckWeaponsIsWorking(t)){
            t.SetValue<float>("Range",1);
            continue;
        }
        if(t.WorldMatrix.Up.Dot(targetPos - (t.GetPosition() + (t.WorldMatrix.Down * 2))) < 0){
            t.SetValue<float>("Range",1);
            continue;
        }
        t.SetValue<float>("Range",800);
        count++;
    }
}

private void FireWeapon()
{
    int count = 0;
    foreach(IMyUserControllableGun w in weaponList){
        if(count >= maxFixedWeaponCount){
            w.ApplyAction("Shoot_Off");
            continue;
        }
        w.ApplyAction("ShootOnce");
        w.ApplyAction("Shoot_On");
        if(restrictFixedWeaponCount){
            count++;
        }
    }
    if(sequencedFire){
        if(sequencedWeaponList.Count == 0){
            return;
        }
        fireDelay += currentTime;
        if(fireDelay < fireInterval){
            return;
        }
        sequencedWeaponList[sequenceStep].ApplyAction("ShootOnce");
        sequenceStep++;
        if(sequenceStep >= sequencedWeaponList.Count){
            sequenceStep = 0;
        }
        fireDelay = 0;
        return;
    }
    foreach(IMyUserControllableGun w in sequencedWeaponList){
        w.ApplyAction("ShootOnce");
        w.ApplyAction("Shoot_On");
    }
}

private void StopWeapon()
{
    foreach(IMyUserControllableGun w in weaponList){
        w.ApplyAction("Shoot_Off");
    }
    foreach(IMyUserControllableGun w in sequencedWeaponList){
        w.ApplyAction("Shoot_Off");
    }
}

public class ThrustAxis
{
    public Base6Directions.Direction Direction {get;}
    public Vector3 DirectionVector {get;}
    public List<IMyThrust> ThrustsForward {get; set;}
    public List<IMyThrust> ThrustsBackward {get; set;}

    public ThrustAxis(Base6Directions.Direction Direction,Vector3D DirectionVector)
    {
        this.Direction = Direction;
        this.DirectionVector = DirectionVector;
        ThrustsForward = new List<IMyThrust>();
        ThrustsBackward = new List<IMyThrust>();
    }

    public bool Add(IMyThrust thrust,IMyTerminalBlock reference)
    {
        if(reference.CubeGrid == thrust.CubeGrid && reference.Orientation.TransformDirectionInverse(thrust.Orientation.Forward) == this.Direction){
            ThrustsForward.Add(thrust);
            return true;
        }
        if(reference.CubeGrid == thrust.CubeGrid && reference.Orientation.TransformDirectionInverse(thrust.Orientation.Forward) == Base6Directions.GetOppositeDirection(this.Direction)){
            ThrustsBackward.Add(thrust);
            return true;
        }
        return false;
    }

    public void Run(Vector3D DestinationDir,Vector3D Input,Vector3D velocity,Vector3D gravity,float mass,bool hydrogen,double mergin,float currentTime)
    {
        double targetSpeed = Input.Dot(this.DirectionVector);
        double SpeedDir = velocity.Dot(this.DirectionVector);
        float reqHoverOutput = gravity != Vector3D.Zero ? (float)(-gravity.Dot(this.DirectionVector) * mass) : 0;
    //Total Output
        float totalForward = GetTotalMaxThrust(ThrustsForward,hydrogen);
        float totalBackward = GetTotalMaxThrust(ThrustsBackward,hydrogen);
    //Requir hovering output
        float reqHoverForward = -reqHoverOutput / totalForward * 100;
        float reqHoverBackward = reqHoverOutput / totalBackward * 100;
    //Stop distance
        double relatedDistance = Math.Abs(DestinationDir.Dot(this.DirectionVector));
        double stopDistanceForward = GetStopDistance(ThrustsForward,mass,SpeedDir,hydrogen);
        double stopDistanceBackward = GetStopDistance(ThrustsBackward,mass,SpeedDir,hydrogen);

        float diff,input;
        if(relatedDistance < 10 || Math.Abs(targetSpeed) < 0.001 || (targetSpeed > 0 && relatedDistance <= stopDistanceForward) || (targetSpeed < 0 && relatedDistance <= stopDistanceBackward)){//Brake
            OnOff(ThrustsForward,0,hydrogen,true);
            OnOff(ThrustsBackward,0,hydrogen,true);
        }else{
            diff = (float)(targetSpeed - SpeedDir);
            if(Math.Abs(diff) < mergin){//Glide
                OnOff(ThrustsForward,reqHoverForward,hydrogen,false);
                OnOff(ThrustsBackward,reqHoverBackward,hydrogen,false);
            }else if(diff < 0){//Adjust speed
                input = ((-diff * mass) / currentTime) / totalForward * 100;
                OnOff(ThrustsForward,reqHoverForward,hydrogen,true,input);
                OnOff(ThrustsBackward,0,hydrogen,false);
            }else if(diff > 0){
                input = ((diff * mass) / currentTime) / totalBackward * 100;
                OnOff(ThrustsForward,0,hydrogen,false);
                OnOff(ThrustsBackward,reqHoverBackward,hydrogen,true,input);
            }
        }
    }

    private void OnOff(List<IMyThrust> thrusts,float hoveringForce,bool hydrogen,bool on,float thrustOverride = 0)
    {
        foreach(IMyThrust t in thrusts){
            if(on){
                if(t.CustomData.Contains("offbyspeedlimitter")){
                    t.ApplyAction("OnOff_On");
                    t.CustomData.Replace("offbyspeedlimitter","");
                }
                if(thrustOverride == 0){
                    ThrustPercent(t,0);
                }else{
                    ThrustPercent(t,thrustOverride + hoveringForce);
                }
                continue;
            }
            if(hoveringForce > 0){
                if(t.CustomData.Contains("offbyspeedlimitter")){
                    t.ApplyAction("OnOff_On");
                    t.CustomData.Replace("offbyspeedlimitter","");
                }
                ThrustPercent(t,hoveringForce);
                continue;
            }
            t.ApplyAction("OnOff_Off");
            if(!t.CustomData.Contains("offbyspeedlimitter"))t.CustomData += "offbyspeedlimitter";
        }
    }

    private void ThrustPercent(IMyThrust t,float percentage)
    {
        float adjusted = MathHelper.Clamp(percentage,0,100);
        t.ApplyAction("OnOff_On");
        t.ThrustOverride = t.MaxThrust * (adjusted / 100);
    }

    public float GetTotalMaxThrust(List<IMyThrust> thrusts,bool hydrogen)
    {
        float effective = 0;
        foreach(IMyThrust thrust in thrusts){
            if(!hydrogen && thrust.BlockDefinition.SubtypeId.Contains("Hydrogen")){
                 continue;
            }
            effective += thrust.MaxEffectiveThrust;
        }
        return (float)effective;
    }

    public double GetStopDistance(List<IMyThrust> thrusts,float totalMass,double speed,bool hydrogen)
    {
        if(speed == 0){
            return 0;
        }
        return totalMass * (speed * speed) / (2 * GetTotalMaxThrust(thrusts,hydrogen));
    }
}

private Vector3D GetThrustInput(Vector3D DestinationDir,double targetSpeed)
{
    return Vector3D.Normalize(DestinationDir) * targetSpeed;
}

private Vector3D InterceptPoint(IMyTerminalBlock reference,Vector3D myForward,Vector3D targetPos,Vector3D myPos,Vector3D targetVelocity,Vector3D myVelocity,double projectileSpeed,double projectileMaxSpeed,double projectileAccel,double currentTime)
{
    Vector3D targetDir = targetPos - myPos;
    Vector3D relatedVelocity = targetVelocity - myVelocity;
    double w = 0;
    double distance = targetDir.LengthSquared();
    if(projectileAccel == 0){
        w = projectileSpeed;
    }else{
        double ta = (projectileMaxSpeed - projectileSpeed) / projectileAccel;
        double d = projectileSpeed * ta + 0.5 * projectileAccel * ta * ta;//Advanced while accelatiing
        if(distance < d){
            w = projectileSpeed + 0.5 * (projectileMaxSpeed - projectileSpeed) * (distance / d);
        }else{
            double tb = _PrevT - ta;
            double S1 = ta * (projectileSpeed + 0.5 * (projectileMaxSpeed - projectileSpeed)) + tb * projectileMaxSpeed;
            double S2 = (ta + tb) * projectileMaxSpeed;
            w = (S1 / S2) * projectileMaxSpeed;
        }
    }

    double a = Vector3D.Dot(relatedVelocity,relatedVelocity) - (w * w);
    double b = 2 * Vector3D.Dot(relatedVelocity,targetDir);
    double c = Vector3D.Dot(targetDir,targetDir);

    double p = -b / (2 * a);
    double q = (b * b) - 4 * a * c;

    //Cannot solve
    if(q <= 0){
        return targetDir + relatedVelocity;
    }

    q = Math.Sqrt(q) / (2 * a);

    double t1 = p - q;
    double t2 = p + q;

    double tgo = 0;
    if (t1 > t2 && t2 > 0){
        tgo = t2;
    }
    else{
        tgo = t1;
    }
    if(Double.IsNaN(tgo))return targetDir + relatedVelocity;
    float error = GetError(reference,myPos,targetPos,targetPos + relatedVelocity * tgo);
    bias += LeadBiasPID.Run(error,(float)currentTime);
    bias = MathHelper.Clamp(bias,-20,20);if(Double.IsNaN(bias))bias = 0;
    _PrevT = tgo;

    return targetDir + relatedVelocity * tgo * bias;
}

private float GetError(IMyTerminalBlock reference,Vector3D myPos,Vector3D targetPos,Vector3D preditedPos)
{
    if(prevTargetPos == Vector3D.Zero || prevPredictedPos == Vector3D.Zero){
        prevTargetPos = targetPos;
        prevPredictedPos = preditedPos;
        return 0;
    }
    Vector3D targetMoveDir = prevPredictedPos - prevTargetPos;
    Vector3D errorVector = prevPredictedPos - (myPos + reference.WorldMatrix.Forward * reference.WorldMatrix.Forward.Dot(prevPredictedPos - myPos));
    prevTargetPos = targetPos;
    prevPredictedPos = preditedPos;
    return (float)Vector3D.Normalize(targetMoveDir).Dot(errorVector);
}

public class PIDControl
{
    public float Kp {get;}
    public float Ki {get;}
    public float Kd {get;}
    public float Decay{get;}
    public float DifferencialThreshold{get;}

    private float _IntegralValue = 0;
    private float _PriorValue = 0;

    public PIDControl(float Kp, float Ki, float Kd,float Decay,float DifferencialThreshold)
    {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Decay = Decay;
        this.DifferencialThreshold = DifferencialThreshold;
    }

    public float Run(float _input,float _currentTime)
    {
        if(_currentTime == 0 || _input == 0){
            return _input;
        }
        _IntegralValue = _IntegralValue + (_input * _currentTime);
        if(Single.IsNaN(_IntegralValue)){
            _IntegralValue = 0;
        }

        float differencial = Math.Abs(_input) > DifferencialThreshold ? (_input - _PriorValue) / _currentTime : 0;
        _PriorValue = _input;
        float result =  this.Kp * _input + this.Ki * _IntegralValue + this.Kd * differencial;
        Decaying(_IntegralValue);

        if(Single.IsNaN(result)){
            return _input;
        }

        return result;
    }

    public float Decaying(float input)
    {
        if(input > 0){
            return input - Decay;
        }
        if(input < 0){
            return input + Decay;
        }
        return input;
    }
}

private bool CheckHydrogen()
{
    double tankLevel = 0;
    float iceLevel = 0;
    foreach(var b in gasSupplyList){
        var tank = b as IMyGasTank;
        if(tank != null){
            tankLevel += tank.FilledRatio;
            continue;
        }
        var gen = b as IMyGasGenerator;
        if(gen != null){
            iceLevel += gen.GetInventory().GetItemAt(0).Value.Amount.ToIntSafe();
            continue;
        }
    }
    if(tankLevel == 0 && iceLevel == 0){
        return false;
    }else{
        return true;
    }
}

private Vector2 GetXYAngle(IMyTerminalBlock reference,Vector3D tmVector,Vector3D myPos)
{
//get forward/up/right vector
    Vector3D forwardVector = reference.WorldMatrix.Forward;
    Vector3D upVector = reference.WorldMatrix.Up;
    Vector3D rightVector = reference.WorldMatrix.Right;
//projection tmVector to x,y,z plane
    double projX = Vector3D.Dot(tmVector,forwardVector);
    double projY = Vector3D.Dot(tmVector,upVector);
    double projZ = Vector3D.Dot(tmVector,rightVector);
    //get raw angle
    return new Vector2(projX < 0 ? 0f : (float)Math.Atan2(projY,projX),(float)Math.Atan2(projZ,projX));
}

private Vector2 GetGravityAlingment(IMyTerminalBlock reference,Vector3D gravity)
{
//get forward/up/right vector
    Vector3D downVector = reference.WorldMatrix.Down;
    Vector3D forwardVector = reference.WorldMatrix.Forward;
    Vector3D rightVector = reference.WorldMatrix.Right;

//projection tmVector to x,y,z plane
    double projX = Vector3D.Dot(gravity,downVector);
    double projY = Vector3D.Dot(gravity,forwardVector);
    double projZ = Vector3D.Dot(gravity,rightVector);
    //get raw angle
    return new Vector2((float)Math.Atan2(projY,projX),(float)Math.Atan2(projZ,projX));
}

public void GyroControl(IMyTerminalBlock Referecne,float yaw,float pitch,float roll,bool _override)
{
//create rotation vector
    Vector3 rotationVector = new Vector3(-pitch,yaw,roll);
//convert to reference direction
    Vector3 refRotationVector = Vector3.TransformNormal(rotationVector,Referecne.WorldMatrix);
    foreach(IMyGyro gyro in gyroList){
//translate to gyro direction
        Vector3 localRotationVector = Vector3.TransformNormal(refRotationVector,Matrix.Transpose(gyro.WorldMatrix));

        gyro.GyroOverride = _override;

        gyro.Pitch = localRotationVector.X;
        gyro.Yaw = localRotationVector.Y;
        gyro.Roll = localRotationVector.Z;
    }
}

private bool GetBlocks()
{
    Echo(scriptName);
    if(updateTimer < updateLimit){
        Echo($"Next Update : {updateLimit - updateTimer:#}");
        if(error.Length > 0){
            return true;
        }
        return false;
    }
    Echo("Blocks Update...");
    updateTimer = 0;
    var blockList = GetOwnGridBlock<IMyTerminalBlock>();
    if(blockList.Count == 0){
        error = "**System Failure**\nBlocks Not Found";
        return true;
    }
    //if(blockList.Count == blockCount){
        //return false;
    //}
//if add or lost blocks
    remoteList.Clear();
    gyroList.Clear();
    gasSupplyList.Clear();
    weaponList.Clear();
    sequencedWeaponList.Clear();
    turretList.Clear();
    rotorList.Clear();
    timerList.Clear();
    thrustAxisList = new List<ThrustAxis>{new ThrustAxis(Base6Directions.Direction.Forward,Vector3D.Forward),new ThrustAxis(Base6Directions.Direction.Up,Vector3D.Up),new ThrustAxis(Base6Directions.Direction.Left,Vector3D.Left)};
//sort blocks
    List<IMyTerminalBlock> thrustList = new List<IMyTerminalBlock>();
    foreach(var b in blockList){
        if(b is IMyRemoteControl && b.CustomData.Contains(remoteName)){
            remoteList.Add(b);
        }else if(b is IMyGyro && b.CubeGrid == Me.CubeGrid){
            gyroList.Add(b);
        }else if(b is IMyThrust && b.CubeGrid == Me.CubeGrid){
            thrustList.Add(b);
        }else if(((b is IMyGasTank && b.BlockDefinition.SubtypeId.Contains("Hydrogen")) || b is IMyGasGenerator) && b.CubeGrid == Me.CubeGrid){
            gasSupplyList.Add(b);
        }else if(b is IMyUserControllableGun && CheckWeaponsIsWorking(b as IMyUserControllableGun) && b.CubeGrid == Me.CubeGrid){
            if(b is IMyLargeTurretBase){
                turretList.Add(b);
            }else if(b.CustomData.Contains(sequencedName)){
                sequencedWeaponList.Add(b);
            }else{
                weaponList.Add(b);
            }
        }else if(b is IMyMotorStator && b.CustomData.Contains(rotorName)){
            rotorList.Add(b);
        }else if(b is IMyTimerBlock && b.CustomData.Contains(timerName)){
            timerList.Add(b);
        }else if(b is IMyParachute){
            var para = b as IMyParachute;
            para.ApplyAction("OnOff_Off");
            para.SetValue<bool>("AutoDeploy",true);
            para.SetValue<float>("AutoDeployHeight",deployHeight);
        }
    }

//block check
    if(remoteList.Count == 0){//critical failure
        error = $"**System Failure**\nRemote Control Block Not Found.\nPlease write \"{remoteName}\" into RC block Custom Data";
        return true;
    }
    //blockCount = blockList.Count;
    foreach(IMyThrust t in thrustList){
        foreach(ThrustAxis axis in thrustAxisList){
            if(axis.Add(t,remote == null ? remoteList[0] : remote)){
                break;
            }
        }
    }
    Sort(rotorList,rotorName);
    ConfigHandler(false);
    if(!start)YawPID = new PIDControl(Kp,Ki,Kd,Decay,DifferencialThreshold);
    if(!start)PitchPID = new PIDControl(Kp,Ki,Kd,Decay,DifferencialThreshold);
    if(!start)RollPID = new PIDControl(Kp,Ki,Kd,Decay,DifferencialThreshold);
    if(!start)LeadBiasPID = new PIDControl(LeadBias_Kp,LeadBias_Ki,LeadBias_Kd,LeadBias_Decay,LeadBias_DifferencialThreshold);
    return false;
}

private void Sort(List<IMyTerminalBlock> sortingList,string _Name)
{
    var blockList = new List<IMyTerminalBlock>();
    int largestNumber = 0;
    int currentNumber = 0;
//find largest number rocket launcher
    foreach(var r in sortingList){
        string[] cdSplit = r.CustomData.Split(';');
        foreach(var str in cdSplit){
            if(!str.Contains(_Name)){
                continue;
            }
            Int32.TryParse(str.Replace(_Name,""),out currentNumber);
            if(largestNumber < currentNumber){
                largestNumber = currentNumber;
                break;
            }
        }
    }
//sort
    for(int i = 1;i < largestNumber + 1;i++){
        foreach(var b in sortingList){
            if(!b.CustomData.Contains(_Name + i.ToString())){
                continue;
            }
            blockList.Add(b);
            break;
        }
    }
    sortingList = new List<IMyTerminalBlock>(blockList);
}

private List<IMyTerminalBlock> GetOwnGridBlock<T>(Func<IMyTerminalBlock, bool> collect = null) where T : class, IMyTerminalBlock
{
    var blockList = new List<IMyTerminalBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyMechanicalConnectionBlock>(blockList);
    HashSet<IMyCubeGrid> CubeGridSet = new HashSet<IMyCubeGrid>();
    CubeGridSet.Add(Me.CubeGrid);
    bool continueLoop;
    IMyMechanicalConnectionBlock block;
//get all CubeGrid connected on ship
    do{
        continueLoop = false;
        for(int i = 0;i < blockList.Count;i++){
            block = blockList[i] as IMyMechanicalConnectionBlock;
            if(CubeGridSet.Contains(block.CubeGrid) || CubeGridSet.Contains(block.TopGrid)){
                CubeGridSet.Add(block.CubeGrid);
                CubeGridSet.Add(block.TopGrid);
                blockList.Remove(blockList[i]);
                continueLoop = true;
            }
        }
    }
    while(continueLoop);

//get filtered block
    blockList.Clear();
    GridTerminalSystem.GetBlocksOfType<T>(blockList,b => CubeGridSet.Contains(b.CubeGrid) && (collect == null || collect(b)));
    return blockList;
}

private void ConfigHandler(bool initial)
{
    ini.Clear();
    MyIniParseResult result;
//---initialize---//
    if(!Me.CustomData.Contains(scriptName) || !ini.TryParse(Me.CustomData,out result)){
        ini.Set(scriptName,"startByCommand",startByCommand);
        ini.Set(scriptName,"dropDistance",dropDistance);
        ini.Set(scriptName,"dropInterval",dropInterval);
        ini.Set(scriptName,"deployHeight",deployHeight);
        ini.Set(scriptName,"weaponRange",weaponRange);
        ini.Set(scriptName,"muzzleVelocity",muzzleVelocity);
        ini.Set(scriptName,"maxMuzzleVelocity",maxMuzzleVelocity);
        ini.Set(scriptName,"projAcceration",projAcceration);
        ini.Set(scriptName,"sequencedFire",sequencedFire);
        ini.Set(scriptName,"fireCycle",fireCycle);
        ini.Set(scriptName,"restrictFixedWeaponCount",restrictFixedWeaponCount);
        ini.Set(scriptName,"maxFixedWeaponCount",maxFixedWeaponCount);
        ini.Set(scriptName,"restrictTurretCount",restrictTurretCount);
        ini.Set(scriptName,"maxTurretCount",maxTurretCount);
        ini.Set(scriptName,"Kp",Kp);
        ini.Set(scriptName,"Ki",Ki);
        ini.Set(scriptName,"Kd",Kd);
        ini.Set(scriptName,"Decay",Decay);
        ini.Set(scriptName,"DifferencialThreshold",DifferencialThreshold);
        ini.Set(scriptName,"LeadBias_Kp",LeadBias_Kp);
        ini.Set(scriptName,"LeadBias_Ki",LeadBias_Ki);
        ini.Set(scriptName,"LeadBias_Kd",LeadBias_Kd);
        ini.Set(scriptName,"LeadBias_Decay",LeadBias_Decay);
        ini.Set(scriptName,"LeadBias_DifferencialThreshold",LeadBias_DifferencialThreshold);
        ini.Set(scriptName,"randomConfig",randomConfig);

        ini.Set(scriptName,"flightMode",flightMode.ToString());
        ini.Set(scriptName,"active",active);
        Me.CustomData = ini.ToString();
        return;
    }
//---config section---//
    startByCommand = ini.Get(scriptName,"startByCommand").ToBoolean();
    dropDistance = ini.Get(scriptName,"dropDistance").ToDouble();
    dropInterval = ini.Get(scriptName,"dropInterval").ToDouble();
    deployHeight = ini.Get(scriptName,"deployHeight").ToSingle();
    weaponRange = ini.Get(scriptName,"weaponRange").ToDouble();
    muzzleVelocity = ini.Get(scriptName,"muzzleVelocity").ToDouble();
    maxMuzzleVelocity = ini.Get(scriptName,"maxMuzzleVelocity").ToDouble();
    projAcceration = ini.Get(scriptName,"projAcceration").ToDouble();
    sequencedFire = ini.Get(scriptName,"sequencedFire").ToBoolean();
    fireCycle = ini.Get(scriptName,"fireCycle").ToDouble();
    restrictFixedWeaponCount = ini.Get(scriptName,"restrictFixedWeaponCount").ToBoolean();
    maxFixedWeaponCount = ini.Get(scriptName,"maxFixedWeaponCount").ToInt32();
    restrictTurretCount = ini.Get(scriptName,"restrictTurretCount").ToBoolean();
    maxTurretCount = ini.Get(scriptName,"maxTurretCount").ToInt32();
    Kp = ini.Get(scriptName,"Kp").ToSingle();
    Ki = ini.Get(scriptName,"Ki").ToSingle();
    Kd = ini.Get(scriptName,"Kd").ToSingle();
    Decay = ini.Get(scriptName,"Decay").ToSingle();
    DifferencialThreshold = ini.Get(scriptName,"DifferencialThreshold").ToSingle();
    LeadBias_Kp = ini.Get(scriptName,"LeadBias_Kp").ToSingle();
    LeadBias_Ki = ini.Get(scriptName,"LeadBias_Ki").ToSingle();
    LeadBias_Kd = ini.Get(scriptName,"LeadBias_Kd").ToSingle();
    LeadBias_Decay = ini.Get(scriptName,"LeadBias_Decay").ToSingle();
    LeadBias_DifferencialThreshold = ini.Get(scriptName,"LeadBias_DifferencialThreshold").ToSingle();
    randomConfig = ini.Get(scriptName,"randomConfig").ToBoolean();

//---variable section---//
    if(initial){
        //load variable data

        FlightMode fmode = flightMode;
        if(!Enum.TryParse(ini.Get(scriptName,"flightMode").ToString(),out flightMode)){
            flightMode = fmode;
        }

        active = ini.Get(scriptName,"active").ToBoolean();
    }else{
        //save variable data
        ini.Set(scriptName,"flightMode",flightMode.ToString());
        ini.Set(scriptName,"active",active);
    }

    Me.CustomData = ini.ToString();
}

private void AntennaNameChanger(string message)//For debug
{
    var list = GetOwnGridBlock<IMyRadioAntenna>();
    list.ForEach(b => b.CustomName = message);
}