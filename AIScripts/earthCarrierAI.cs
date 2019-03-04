const short SPAWNED = 0;
const short FLYING_TO_START = 1;
const short SCANNING = 2;
const short ENEMY_FOUND = 3;
const short FLYING_TO_BOMBARD = 4;
const short BOMBARDING = 5;
const short RETREAT = 6;
const short RETREATING = 7;

const short RESPONSE_WEAK = 1;
const short RESPONSE_MEDIUM = 2;
const short RESPONSE_STRONG = 3;

const int UPDATE_FREQUENCY = 100;
const int EXECUTION_FREQUENCY = UPDATE_FREQUENCY * 10;
const int DETECT_COOLDOWN = EXECUTION_FREQUENCY * 20;
const int REFILL_FREQUENCY = UPDATE_FREQUENCY * 200;

const string RESPONSE_WEAK_NAME = "Insignificant threat sighted. Dispatching response...";
const string RESPONSE_MEDIUM_NAME = "Notable threat sighted. Dispatching response...";
const string RESPONSE_STRONG_NAME = "Radiation detected. Dispatching strong response...";

double visualRange = 3000;   
int speedLimit = 50;
long tickTimer = 0;
long lastDispatchedTime = 0 - DETECT_COOLDOWN;

bool planetCentreSet = false;
short status = SPAWNED;

Vector3D planetCentreGPS;

string lcdName = "LCD Panel";
string remoteControlName = "(MAIN) Remote Control Forward";
string backupRemoteControlName = "(CRUISE) Remote Control Forward";
string ammoCargoName = "Ammo Small Cargo Container";
string fuelCargoName = "Fuel Small Cargo Container";
string bombardLauncherGroupName = "Bombard Launchers";
string bombardGyroName = "Gyroscope Spin";
string forwardMarkerName = "Battery Forwards";
string backwardMarkerName = "Battery Backwards";
string raycastCameraName = "Recon Camera";
IMyRemoteControl remoteControl = null;
IMyCameraBlock reconCamera = null;
Dictionary<long, short> activeTargets = new Dictionary<long, short>();

public Program() {
	Runtime.UpdateFrequency = UpdateFrequency.Update10;
}

void Main() {
	tickTimer += UPDATE_FREQUENCY;
	if(tickTimer % EXECUTION_FREQUENCY == 0) {
		run();
	}
}

void run() {
	// SendChatMessage("TickTimer:" + tickTimer, "Space Pirates");
	if(lastDispatchedTime + DETECT_COOLDOWN > tickTimer) {
		//response has been dispatched too recently. Do not Scan. 
		return;
	}
	remoteControl = GridTerminalSystem.GetBlockWithName(remoteControlName) as IMyRemoteControl;
	if(remoteControl == null) {
		Echo("no remote control found, aborting");
		return;
	}
	Echo("Remote control found.");
	reconCamera = getCamera();
	if(reconCamera == null) {
		Echo("Recon camera not found.");
	}
	else {
		reconCamera.EnableRaycast = true;
	}
	long targetId = detectNearbyEnemies();
	if(targetId != 0) {
		Echo("targetId:" + targetId + " response:" + activeTargets[targetId]);
		dispatchResponse(targetId, activeTargets[targetId]);
	}
	refillIfApplicable();
	printStatus();

}

long detectNearbyEnemies() {
	//TODO: probably go for closest or radioactive first
	List<long> enemyGridIds = GetAllEnemyGrids("None", visualRange);
	Echo("Found " + enemyGridIds.Count + " enemy grids.\n");
	short response = 0;
	long target = 0;
	foreach (long gridId in enemyGridIds) {
		if(TargetPowered(gridId)) {
			Echo("target " + gridId + " is powered");
			long activeTargetBlock = findReactors(gridId);
			Echo("activeTargetBlock:" + activeTargetBlock);
			if(activeTargetBlock != 0) {
				Echo("target " + gridId + " has radiation");
				response += RESPONSE_STRONG;
				target = activeTargetBlock;
			}
			else {
				Echo("casting ray at grid " + gridId);
				MyDetectedEntityType entityType = castRayAt(gridId);
				Echo("Entity type:" + entityType);
				if(entityType == MyDetectedEntityType.SmallGrid) {
					Echo("target " + gridId + " is a visible small grid");
					response += RESPONSE_WEAK;
					target = gridId;
				}
				else if(entityType == MyDetectedEntityType.LargeGrid) {
					Echo("target " + gridId + " is a visible LARGE grid");
					response += RESPONSE_MEDIUM;
					target = gridId;
				}
			}
			if(target != 0) {
				if(TargetCanFly(gridId)) {
					Echo("target " + gridId + " can fly");
					response += RESPONSE_WEAK;
				}
				activeTargets.Add(target, response);
				return target;
			}
		}
	}
	Echo("No valid targets found.");
	return target;
}

MyDetectedEntityType castRayAt(long gridId) {
	//TODO - check for empty enum (this will either be null or set to empty)
	if(reconCamera != null) {
		Echo("Recon camera found");
		Echo("Range: " + reconCamera.AvailableScanRange.ToString());
		Vector3D targetPos = GetTrackedEntityPosition(gridId);
		if(reconCamera.CanScan(targetPos)) {
			MyDetectedEntityInfo mdei = reconCamera.Raycast(targetPos);
			Echo("MDEI:" + mdei);
			return mdei.Type;
		}
		Echo("Cannot cast ray");
	}


	return MyDetectedEntityType.None;
}

long findReactors(long gridId) {
	long blockId = GetTargetShipSystem(gridId, "MyObjectBuilder_Reactor");
	return blockId;
}

void dispatchResponse(long gridId, short response) {
	String antennaName = null;
	if(response <= RESPONSE_WEAK) {
			antennaName = RESPONSE_WEAK_NAME;
	}
	else if(response <= RESPONSE_MEDIUM) {
		antennaName = RESPONSE_MEDIUM_NAME;
	}
	else {
		antennaName = RESPONSE_STRONG_NAME;
	}
	if(antennaName != null) {
		List<IMyRadioAntenna> antennae = new List<IMyRadioAntenna>();
		GridTerminalSystem.GetBlocksOfType(antennae);
		if(antennae.Any()) {
			IMyRadioAntenna antenna = antennae[0];
			antenna.CustomName = antennaName;
			antenna.ApplyAction("OnOff_On");
			if(!antenna.GetValue<bool>("EnableBroadCast")) {
				antenna.ApplyAction("EnableBroadCast");
			}
		}
	}
	//NEED TO CHECK IF THIS CAN SPAWN prefabs with drone behaviour or only prefabs with no behaviour. If the latter, probably need to use antennaes instead
	//SpawnReinforcements(string spawnType, string spawnName, string spawnFaction, bool mustSpawnAll, Vector3D spawnCoords, Vector3D forwardDirection, Vector3D upDirection, Vector3D spawnVelocity)
}

IMyCameraBlock getCamera() {
	//TODO error checking
	return GridTerminalSystem.GetBlockWithName(raycastCameraName) as IMyCameraBlock;
}

void refillIfApplicable() {
	if(tickTimer % REFILL_FREQUENCY == 0) {
		IMyTerminalBlock fuelCargo = GridTerminalSystem.GetBlockWithName(fuelCargoName);
		if(fuelCargo == null) {
			Echo("Warning! Fuel cargo container not found.");
			return;
		}
		bool createdSuccessfully = CreateItemInInventory("MyObjectBuilder_Ingot", "Uranium", 20, fuelCargo.GetId());
		if(!createdSuccessfully) {
			Echo("Failed to refill reactor fuel");
		}
	}
}

void printStatus() {
	Echo("STATUS: " + status);
	Echo("planetCentreSet:" + planetCentreSet);
}

bool getPlanetCentre(ref Vector3D coords) {
	//Get planetary centre gps from control block     
	IMyShipController control = GridTerminalSystem.GetBlockWithName(remoteControlName) as IMyShipController;  
	if(control == null) {  
		Echo("Control block not found");  
		return false;  
	}  
	Vector3D planetCentre = new Vector3D(0,0,0);     
	bool insideGrav = control.TryGetPlanetPosition(out planetCentre);  
	   
	if (insideGrav) {
		coords = planetCentre;
		return true;
	}
	else {
		return false;
	}
	
}

bool interpolateByDistance(Vector3D a, Vector3D b, double distance, ref Vector3D coords) {
	Vector3D aNormalizedToB = Vector3D.Normalize(a - b);       
	coords =  a + aNormalizedToB * distance;
	return true;
}

bool getVectorPerpendicularToTarget(Vector3D target, Vector3D planetCentre, double xOffset, double yOffset, double zOffset, ref Vector3D coords) {
	//does not work
	Vector3D targetNormalizedToPlanet = Vector3D.Normalize(target - planetCentre);
	Vector3D offsetNormalizedToTarget = Vector3D.Normalize(new Vector3D(xOffset, yOffset, zOffset) - target);
	coords = target + targetNormalizedToPlanet * offsetNormalizedToTarget;
	//coords =  target + offsetNormalizedToPlanet;
	return true;
}

bool distanceLessThan(Vector3D a, Vector3D b, double dist) {
	return Vector3D.Distance(a, b) < dist;
}

void flyToLocation(Vector3D coords) {
	
	remoteControl.ClearWaypoints();
	remoteControl.AddWaypoint(coords, "Destination");
	remoteControl.FlightMode = FlightMode.OneWay;
	remoteControl.SetAutoPilotEnabled(true);
	remoteControl.SetCollisionAvoidance(true);
	remoteControl.SpeedLimit = speedLimit;
	try {
		IMyRemoteControl backup = GridTerminalSystem.GetBlockWithName(backupRemoteControlName) as IMyRemoteControl;
		if(backup != null) {
			backup.ClearWaypoints();
			backup.SetAutoPilotEnabled(false);
			backup.SetCollisionAvoidance(false);
		}
	}
	catch(Exception e) {
		Echo("Backup Remote Control not found.");
	}
}

List<long> GetAllEnemyGrids(string specificFaction = "None", double distanceToCheck = 15000) {
	
	try{
		
		Me.CustomData = specificFaction + "\n" + distanceToCheck.ToString();
		return Me.GetValue<List<long>>("NpcExtender-GetAllEnemies");
		
	}catch(Exception exc){
		
		Echo("NpcExtender-GetAllEnemy Hard Fail");
		return new List<long>();
		
	}

}

bool TargetIsStatic(long entityId){
	
	try{
		
		Me.CustomData = entityId.ToString();
		return Me.GetValue<bool>("NpcExtender-TargetIsStatic");
		
	}catch(Exception exc){
		
		return false;
		
	}

}

bool TargetPowered(long entityId){
	
	try{
		
		Me.CustomData = entityId.ToString();
		return Me.GetValue<bool>("NpcExtender-TargetPowered");
		
	}catch(Exception exc){
		
		return false;
		
	}

}

int TargetBlockCount(long entityId){
	
	try{
		
		Me.CustomData = entityId.ToString();
		return Me.GetValue<int>("NpcExtender-TargetBlockCount");
		
	}catch(Exception exc){
		
		return 0;
		
	}

}

Vector3D GetTrackedEntityPosition(long entityId){
	
	try{
		
		Me.CustomData = entityId.ToString();
		return Me.GetValue<Vector3D>("NpcExtender-TrackEntity");
		
	}catch(Exception exc){
		
		return new Vector3D(0,0,0);
		
	}

}

bool SendChatMessage(string message, string author, string audioClip = ""){
	
	double broadcastDistance = 0;
	var antennaList = new List<IMyRadioAntenna>();
	GridTerminalSystem.GetBlocksOfType<IMyRadioAntenna>(antennaList);
	
	foreach(var antenna in antennaList){
		
		if(antenna.IsFunctional == false || antenna.Enabled == false || antenna.EnableBroadcasting == false){
			
			continue;
			
		}
		
		var antennaRange = (double)antenna.Radius;
		
		if(antennaRange > broadcastDistance){
			
			broadcastDistance = antennaRange;
			
		}
	
	}
	
	if(broadcastDistance == 0){
		
		return false;
		
	}
	
	try{
		
		string sendData = message + "\n" + author + "\n" + broadcastDistance.ToString() + "\n" + audioClip;
		Me.CustomData = sendData;
		return Me.GetValue<bool>("NpcExtender-ChatToPlayers");
		
	}catch(Exception exc){
		
		return false;
		
	}

}

bool CreateItemInInventory(string type, string subtype, float amount, long blockEntityId){

	Me.CustomData = type + "\n" + subtype + "\n" + amount.ToString() + "\n" + blockEntityId.ToString();
	
	try{
		
		return Me.GetValue<bool>("NpcExtender-CreateItemInInventory");
		
	}catch(Exception exc){
		
		return false;
		
	}
	
}

bool AttemptDespawn(){
	
	try{
		
		return Me.GetValue<bool>("NpcExtender-DespawnDrone");
		
	}catch(Exception exc){
		
		return false;
		
	}
			
}

bool SpawnReinforcements(string spawnType, string spawnName, string spawnFaction, bool mustSpawnAll, Vector3D spawnCoords, Vector3D forwardDirection, Vector3D upDirection, Vector3D spawnVelocity){
	
	string spawnData = spawnType + "\n";
	spawnData += spawnName + "\n";
	spawnData += spawnFaction + "\n";
	spawnData += mustSpawnAll.ToString() + "\n";
	spawnData += spawnCoords.ToString() + "\n";
	spawnData += forwardDirection.ToString() + "\n";
	spawnData += upDirection.ToString() + "\n";
	spawnData += spawnVelocity.ToString();
	
	try{
		
		Me.CustomData = spawnData;
		return Me.GetValue<bool>("NpcExtender-SpawnReinforcements");
		
	}catch(Exception exc){
		
		return false;
		
	}
	
}

bool TargetCanFly(long entityId){
	
	try{
		
		Me.CustomData = entityId.ToString();
		return Me.GetValue<bool>("NpcExtender-TargetCanFly");
		
	}catch(Exception exc){
		
		return false;
		
	}
	
}

bool TargetIsBroadcasting(long entityId, bool checkAntennas = true, bool checkBeacons = true){
	
	try{
		
		Me.CustomData = entityId.ToString() + "\n" + checkAntennas.ToString() + "\n" + checkBeacons.ToString();
		return Me.GetValue<bool>("NpcExtender-TargetIsBroadcasting");
		
	}catch(Exception exc){
		
		return false;
		
	}

}

Vector3D GetNearestPlanetPosition(Vector3D checkCoords){

	try{
		
		Me.CustomData = checkCoords.ToString();
		return Me.GetValue<Vector3D>("NpcExtender-GetNearestPlanetPosition");
		
	}catch(Exception exc){
		
		return Vector3D.Zero;
		
	}

}

long GetTargetShipSystem(long targetEntityId, string targetSystems = "MyObjectBuilder_Reactor"){
	
	/*var systemsList = new List<string>();
	systemsList.Add("Clang"); //Rotors / Pistons / Wheels
	systemsList.Add("Communications"); //Antennas / Beacons / Laser Antennas
	systemsList.Add("Controller"); //Seats / Remote Controls / Etc
	systemsList.Add("Power"); //Reactors / Batteries / Solar
	systemsList.Add("Production"); //Refineries / Assemblers / Gas Generators
	systemsList.Add("Propulsion"); //Thrusters
	systemsList.Add("Weapons"); //Turrets / Fixed Weapons
	
	if(systemsList.Contains(targetSystems) == false){
		
		return 0;
		
	}*/
	
	try{
		
		Me.CustomData = targetEntityId.ToString() + "\n" + targetSystems;
		return Me.GetValue<long>("NpcExtender-GetTargetShipSystem");
		
	}catch(Exception exc){
		
		return 0;
		
	}

}