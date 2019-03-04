/*NPC Rotor Turret FCS by Sunoko*/

//=============BASIC SETTINGS=============//
//yaw rotor name
const string yawRotorName = "yaw";
//pitch rotor name
const string pitchRotorName = "pitch";
//left name
const string leftName = "left";
//right name
const string rightName = "right";
//upside down yaw rotor name
const string upsidedownName = "upsidedown";
//refernce rotor name
const string turretRefRotorName = "reference";
//sync rotor name
const string turretSyncRotorName = "sync";
//remote control name
const string rcName = "turret";
//los check camera name
const string cameraName = "los";
//weapon name use for calculate turret center position
const string refName = "ref";
//timer name
const string timerName = "timer";
//get only tagged weapon
bool onlyTaggedWeapon = false;
//weapon name
const string weaponName = "weapon";
//weapons fire distance
double aimingDistance = 800;
//fireing weapon accuracy low value = high accurate
float accuracy = 10f;
//turret rotation speed limit
float speedLimit = 10f;
//Weapon muzzle velocity
double muzzleVelocity = 400;
//Weapon muzzle velocity cap
double maxMuzzleVelocity = 0;
//Projectile Acceration
double projAcceration = 0;
//enable seqenced fire
bool sequencedFire = false;
//sequenced weapon name
const string sequencedName = "sequenced";
//fire interval
double fireCycle = 1;
//start by distance or trigger by other
bool startByDistance = false;
//activate command
const string commandActive = "Active";
//========================================//

//--------DO NOT EDIT BELLOW CODE---------//
//-------------other settings-------------//
//script name
const string scriptName = "NPC Rotor Turret FCS";
//start distance when "startByDistance" is true
double startDistance = 1000;
//Default PID Gain
float Kp = 0.5f;
float Ki = 0f;
float Kd = 0.01f;
//Integral decay factor
float Decay = 0.01f;
//Adding differencial threshold
float DifferencialThreshold = 0;//nearly 30 degree
//Target leading bias PID Gain
float LeadBias_Kp = 0.002f;
float LeadBias_Ki = 0f;
float LeadBias_Kd = 0.001f;
//Integral decay factor
float LeadBias_Decay = 0.01f;
//Adding differencial threshold
float LeadBias_DifferencialThreshold = 0;
//---------------block list---------------//
List<IMyTerminalBlock> remoteList = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> pitchRotorList = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> yawRotorList = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> weaponList = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> sequencedWeaponList = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> cameraList = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> beaconList = new List<IMyTerminalBlock>();

//int blockCount = 0;
//----------------variable----------------//
MyIni ini = new MyIni();
const float RadToDegF = (float)(180 / Math.PI);
IMyRemoteControl remote;
IMyMotorStator yawRotorReference;
IMyMotorStator pitchRotorReference;
PIDControl YawPID;
PIDControl PitchPID;
PIDControl LeadBiasPID;
Vector3D prevDir = new Vector3D(0,0,0);
Vector3D playerPos = new Vector3D(0,0,0);
Vector3D previousPos = new Vector3D(0,0,0);
Vector3D playerVelocity = new Vector3D(0,0,0);
Vector3D prevVelocity = new Vector3D(0,0,0);
Vector3D targetPos = new Vector3D(0,0,0);
Vector3D prevTargetPos = Vector3D.Zero;
Vector3D prevPredictedPos = Vector3D.Zero;
Vector3D myPos = new Vector3D(0,0,0);
MyDetectedEntityInfo info;
bool fire = true;
bool active = false;
bool start = false;
int _RaycastStep = 0;
double fireInterval = 0;
int sequenceStep = 0;
double fireDelay = 0;
double _PrevT = 2;
double bias = 0;
//--run timer--//
const double runPerSec = 15;
const double cycle = 1 / runPerSec;
double currentTime = 0;
//--update timer--//
int updateLimit = 10;
double updateTimer = 0;
string error = "";

public Program()//Program() is run once at loading PB
{
    updateTimer = updateLimit + 1;
    Runtime.UpdateFrequency = UpdateFrequency.Update1;
    ConfigHandler(true);
}

public void Main(string argument)
{
    if(!startByDistance && argument == commandActive){
        active = true;
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
    if(startByDistance){
        Echo("Startup Distance : {startDistance}");
    }else{
        Echo($"Active Command Recieved : {active}");
    }
    if(currentTime == 0){
        return;
    }
    fireInterval = fireCycle / sequencedWeaponList.Count;
//get player position and distance
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
    if(!remote.GetNearestPlayer(out playerPos)){
        return;
    }
    myPos = remote.GetPosition();
    double distance = Vector3D.Distance(playerPos,myPos);
    if(startByDistance){
        if(distance > startDistance){
            active = true;
            Reset();
            return;
        }
    }
    if(!active){
        AltitudeSensor();
        Reset();
        return;
    }
    start = true;
    if(previousPos != Vector3D.Zero){
        playerVelocity = (playerPos - previousPos) / currentTime;
    }
    previousPos = playerPos;

//target leading
    Vector3D myVelocity = remote.GetShipVelocities().LinearVelocity;
    targetPos = InterceptPoint(remote,remote.WorldMatrix.Forward,playerPos,myPos,playerVelocity,myVelocity,muzzleVelocity,maxMuzzleVelocity,projAcceration,currentTime);
//get pitch and yaw angle
    Vector3D _FixedTargetPosition = (myPos - AveragePosition()) + targetPos;
    Vector2 rotation = GetXYAngle(remote,_FixedTargetPosition,myPos);
    float yaw = YawPID.Run(rotation.Y * RadToDegF,(float)currentTime);
    float pitch = PitchPID.Run(rotation.X * RadToDegF,(float)currentTime);
    yaw = (float)MathHelper.Clamp(yaw,-speedLimit,speedLimit);
    pitch = (float)MathHelper.Clamp(pitch,-speedLimit,speedLimit);
//rotor turret control
    float yawAngle = 0;
    if(yawRotorReference != null){
        if(pitchRotorReference.CustomData.Contains(upsidedownName)){
            yawRotorReference.SetValue<float>("Velocity",-yaw);
            yawAngle = -yawRotorReference.Angle;
        }else{
            yawRotorReference.SetValue<float>("Velocity",yaw);
            yawAngle = yawRotorReference.Angle;
        }
    }
    foreach(IMyMotorStator Rotor in yawRotorList){
        if(yawRotorReference != null && yawRotorReference.IsAttached && Rotor.CustomData.Contains(turretSyncRotorName)){
            if(Rotor.CustomData.Contains(upsidedownName)){
                Rotor.SetValue<float>("Velocity",-(yawAngle + Rotor.Angle) * RadToDegF);
            }else{
                Rotor.SetValue<float>("Velocity",(yawAngle - Rotor.Angle) * RadToDegF);
            }
        }else{
            if(Rotor.CustomData.Contains(upsidedownName)){
                Rotor.SetValue<float>("Velocity",-yaw);
            }else{
                Rotor.SetValue<float>("Velocity",yaw);
            }
        }
    }
    float pitchAngle = 0;
    if(pitchRotorReference != null){
        if(pitchRotorReference.CustomData.Contains(leftName)){
            pitchRotorReference.SetValue<float>("Velocity",pitch);
            pitchAngle = pitchRotorReference.Angle;
        }else if(pitchRotorReference.CustomData.Contains(rightName)){
            pitchRotorReference.SetValue<float>("Velocity",-pitch);
            pitchAngle = -pitchRotorReference.Angle;
        }
    }
    foreach(IMyMotorStator Rotor in pitchRotorList){
        if(Rotor.CustomData.Contains(leftName)){
            if(pitchRotorReference != null && pitchRotorReference.IsAttached && Rotor.CustomData.Contains(turretSyncRotorName)){
                Rotor.SetValue<float>("Velocity",(pitchAngle - Rotor.Angle) * RadToDegF);
            }else{
                Rotor.SetValue<float>("Velocity",pitch);
            }
        }else if(Rotor.CustomData.Contains(rightName)){
            if(pitchRotorReference != null && pitchRotorReference.IsAttached && Rotor.CustomData.Contains(turretSyncRotorName)){
                Rotor.SetValue<float>("Velocity",-(pitchAngle + Rotor.Angle) * RadToDegF);
            }else{
                Rotor.SetValue<float>("Velocity",-pitch);
            }
        }
    }
//los check
    if(distance < aimingDistance){
        TrySequentiallyRaycast();
    }
    if(info.Type == MyDetectedEntityType.Planet || info.Relationship.IsFriendly()){
        fire = false;
    }else{
        fire = true;
    }
//fire
    double angle = AngleBetweenRad(remote.WorldMatrix.Forward,_FixedTargetPosition) * RadToDegF;
    if(fire && distance < aimingDistance && angle < accuracy){
        foreach(IMyUserControllableGun w in weaponList){
            w.ApplyAction("ShootOnce");
            w.ApplyAction("Shoot_On");
        }
    //rocket launcher
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
    }else{
        foreach(IMyUserControllableGun w in sequencedWeaponList){
            w.ApplyAction("Shoot_Off");
        }
        foreach(IMyUserControllableGun w in weaponList){
            w.ApplyAction("Shoot_Off");
        }
    }
}

private double AngleBetweenRad(Vector3D a,Vector3D b)
{
    double l = Vector3D.Normalize(a).Dot(Vector3D.Normalize(b));
    return Math.Acos(l);
}

private void AltitudeSensor()
{
    double elevation = 0;
    remote.TryGetPlanetElevation(MyPlanetElevation.Surface, out elevation);
    if(elevation < 10){
        List<IMyTerminalBlock> list = new List<IMyTerminalBlock>();
        GridTerminalSystem.GetBlocksOfType<IMyTimerBlock>(list,b => b.CustomData.Contains(timerName));
        foreach(IMyTimerBlock timer in list){
            if(timer.IsCountingDown){
                continue;
            }
            timer.ApplyAction("Start");
        }
    }
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

private bool TrySequentiallyRaycast()
{
    if(cameraList.Count == 0){
        return false;
    }
    var _Camera = cameraList[_RaycastStep] as IMyCameraBlock;
    _Camera.EnableRaycast = true;
    if(!_Camera.CanScan(aimingDistance)){
        return false;
    }
    Vector3D raycastPoint = _Camera.GetPosition() + ((playerPos - _Camera.GetPosition()) * 1.1) + Vector3D.Normalize(-remote.GetNaturalGravity());
    info = _Camera.Raycast(raycastPoint);

    _RaycastStep++;
     if(_RaycastStep >= cameraList.Count){
        _RaycastStep = 0;
    }

    if(info.Type == MyDetectedEntityType.Planet){
        raycastPoint = _Camera.GetPosition() + ((playerPos - _Camera.GetPosition()) * 1.1);
        info = _Camera.Raycast(raycastPoint);
        _RaycastStep++;
         if(_RaycastStep >= cameraList.Count){
            _RaycastStep = 0;
        }
    }

    return true;
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

private Vector3D AveragePosition()
{
    Vector3D WeaponPosition = new Vector3D(0,0,0);
    int count = 0;
    foreach(var w in weaponList){
        if(w.CustomData.Contains(refName)){
            WeaponPosition += w.GetPosition();
            count++;
        }
    }
    foreach(var r in sequencedWeaponList){
        if(r.CustomData.Contains(refName)){
            WeaponPosition += r.GetPosition();
            count++;
        }
    }
    if(yawRotorReference != null && yawRotorReference.CustomData.Contains(refName)){
        WeaponPosition += yawRotorReference.GetPosition();
        count++;
    }
    if(pitchRotorReference != null && pitchRotorReference.CustomData.Contains(refName)){
        WeaponPosition += pitchRotorReference.GetPosition();
        count++;
    }
    if(count == 0){
        if(weaponList.Count == 0){
            return sequencedWeaponList[0].GetPosition();
        }
        return weaponList[0].GetPosition();
    }
    return WeaponPosition / count;
}

private void Reset()
{
    foreach(IMyUserControllableGun w in weaponList){
        w.ApplyAction("Shoot_Off");
    }
    foreach(IMyMotorStator r in yawRotorList){
        r.SetValue<float>("Velocity",0);
    }
    foreach(IMyMotorStator r in pitchRotorList){
        r.SetValue<float>("Velocity",0);
    }
    if(yawRotorReference != null){
        yawRotorReference.SetValue<float>("Velocity",0);
    }
    if(pitchRotorReference != null){
        pitchRotorReference.SetValue<float>("Velocity",0);
    }
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

public bool GetBlocks()
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
    var blockList = GetOwnGridBlock();
    if(blockList.Count == 0){
        error = "**System Failure**\nBlocks Not Found";
        return true;
    }
    //if(blockList.Count == blockCount){
        //return false;
    //}
//if add or lost blocks
    remoteList.Clear();
    yawRotorList.Clear();
    pitchRotorList.Clear();
    weaponList.Clear();
    sequencedWeaponList.Clear();
    cameraList.Clear();
    beaconList.Clear();
//sort blocks
    foreach(var b in blockList){
        if(b is IMyShipController && b.CustomData.Contains(rcName)){
            remoteList.Add(b);
        }else if(b is IMyMotorStator){
            if(b.CustomData.Contains(yawRotorName)){
                if(b.CustomData.Contains(turretRefRotorName)){
                    yawRotorReference = b as IMyMotorStator;
                }else{
                    yawRotorList.Add(b);
                }
            }else if(b.CustomData.Contains(pitchRotorName)){
                if(b.CustomData.Contains(turretRefRotorName)){
                    pitchRotorReference = b as IMyMotorStator;
                }else{
                    pitchRotorList.Add(b);
                }
            }
        }else if((b is IMyUserControllableGun && !(b is IMyLargeTurretBase) && !b.CustomData.Contains(sequencedName)) && (!onlyTaggedWeapon || (onlyTaggedWeapon && b.CustomData.Contains(weaponName)))){
            weaponList.Add(b);
        }else if((b is IMyUserControllableGun && !(b is IMyLargeTurretBase) && b.CustomData.Contains(sequencedName)) && (!onlyTaggedWeapon || (onlyTaggedWeapon && b.CustomData.Contains(weaponName)))){
            sequencedWeaponList.Add(b);
        }else if(b is IMyCameraBlock && b.CustomData.Contains(cameraName)){
            cameraList.Add(b);
            (b as IMyCameraBlock).EnableRaycast = true;
        }else if(b is IMyBeacon || b is IMyRadioAntenna){
            beaconList.Add(b);
        }
    }
//block check
    if(remoteList.Count == 0){//rc are missing
        error = "**System Failure**\nRemote Control Block Not Found";
        return true;
    }
    if(yawRotorList.Count == 0 && yawRotorReference == null){//rotor are missing
        error = "**Attention**\nYaw Rotor Not Found";
    }
    if(pitchRotorList.Count == 0 && pitchRotorReference == null){//rotor are missing
        error = "**Attention**\nPitch Rotor Not Found";
    }
    if(weaponList.Count == 0 && sequencedWeaponList.Count == 0){//weapon are missing
        error = "**System Failure**\nWeapons Not Found";
        return true;
    }
    //blockCount = blockList.Count;
    error = "";
//set start distance
    if(beaconList.Count > 0){
        if(beaconList[0] is IMyBeacon){
            startDistance = (beaconList[0] as IMyBeacon).Radius;
        }
        if(beaconList[0] is IMyRadioAntenna){
            startDistance = (beaconList[0] as IMyRadioAntenna).Radius;
        }
    }
    ConfigHandler(false);
    if(!start)YawPID = new PIDControl(Kp,Ki,Kd,Decay,DifferencialThreshold);
    if(!start)PitchPID = new PIDControl(Kp,Ki,Kd,Decay,DifferencialThreshold);
    if(!start)LeadBiasPID = new PIDControl(LeadBias_Kp,LeadBias_Ki,LeadBias_Kd,LeadBias_Decay,LeadBias_DifferencialThreshold);
    return false;
}

private List<IMyTerminalBlock> GetOwnGridBlock()
{
    var blockList = new List<IMyTerminalBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyMechanicalConnectionBlock>(blockList);
    HashSet<IMyCubeGrid> CubeGridSet = new HashSet<IMyCubeGrid>();
    CubeGridSet.Add(Me.CubeGrid);
    bool continueLoop;
    IMyMechanicalConnectionBlock block;
//get all CubeGrid on ship
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
    GridTerminalSystem.GetBlocksOfType<IMyTerminalBlock>(blockList,b => CubeGridSet.Contains(b.CubeGrid));
    return blockList;
}

private void ConfigHandler(bool initial)
{
    ini.Clear();
    MyIniParseResult result;
//---initialize---//
    if(!Me.CustomData.Contains(scriptName) || !ini.TryParse(Me.CustomData,out result)){
        ini.Set(scriptName,"startByDistance",startByDistance);
        ini.Set(scriptName,"aimingDistance",aimingDistance);
        ini.Set(scriptName,"muzzleVelocity",muzzleVelocity);
        ini.Set(scriptName,"maxMuzzleVelocity",maxMuzzleVelocity);
        ini.Set(scriptName,"projAcceration",projAcceration);
        ini.Set(scriptName,"accuracy",accuracy);
        ini.Set(scriptName,"speedLimit",speedLimit);
        ini.Set(scriptName,"sequencedFire",sequencedFire);
        ini.Set(scriptName,"fireCycle",fireCycle);
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

        ini.Set(scriptName,"active",active);
        Me.CustomData = ini.ToString();
        return;
    }
//---config section---//
    startByDistance = ini.Get(scriptName,"startByDistance").ToBoolean();
    aimingDistance = ini.Get(scriptName,"aimingDistance").ToDouble();
    muzzleVelocity = ini.Get(scriptName,"muzzleVelocity").ToDouble();
    maxMuzzleVelocity = ini.Get(scriptName,"maxMuzzleVelocity").ToDouble();
    projAcceration = ini.Get(scriptName,"projAcceration").ToDouble();
    accuracy = ini.Get(scriptName,"accuracy").ToSingle();
    speedLimit = ini.Get(scriptName,"speedLimit").ToSingle();
    sequencedFire = ini.Get(scriptName,"sequencedFire").ToBoolean();
    fireCycle = ini.Get(scriptName,"fireCycle").ToDouble();
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
//---variable section---//
    if(initial){
        //load variable data
        active = ini.Get(scriptName,"active").ToBoolean();
    }else{
        //save variable data
        ini.Set(scriptName,"active",active);
    }

    Me.CustomData = ini.ToString();
}