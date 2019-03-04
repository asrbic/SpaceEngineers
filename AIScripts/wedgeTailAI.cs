const short RESPONSE_WEAK = 1;
const short RESPONSE_MEDIUM = 2;
const short RESPONSE_STRONG = 3;

const int UPDATE_FREQUENCY = 100;
const int EXECUTION_FREQUENCY = UPDATE_FREQUENCY * 10;
const int DETECT_COOLDOWN = EXECUTION_FREQUENCY * 30;
const int REFILL_FREQUENCY = UPDATE_FREQUENCY * 200;
const int DISPATCH_DURATION = EXECUTION_FREQUENCY * 7;

const string RESPONSE_WEAK_NAME = "Insignificant threat sighted. Dispatching response...";
const string RESPONSE_MEDIUM_NAME = "Notable threat sighted. Dispatching response...";
const string RESPONSE_STRONG_NAME = "Radiation detected. Dispatching strong response...";
const string NORMAL_ANTENNA_NAME = "Wedge Tailed Scout";

double visualRange = 3000;   
int speedLimit = 50;
long tickTimer = 0;
long lastDispatchedTime = 0 - DETECT_COOLDOWN;



string remoteControlName = "Remote Control";
IMyRemoteControl remoteControl = null;
List<IMyCameraBlock> cameras = null;
Dictionary<long, short> activeTargets = new Dictionary<long, short>();
HashSet<long> knownNPCGridIds = new HashSet<long>();
HashSet<long> knownPlayerGridIds = new HashSet<long>();
public Program() {
	Runtime.UpdateFrequency = UpdateFrequency.Update10;
	setAntennaName(NORMAL_ANTENNA_NAME);
}

void Main() {
	if(tickTimer % EXECUTION_FREQUENCY == 0) {
		run();
	}
	tickTimer += UPDATE_FREQUENCY;
}

void run() {
	//SendChatMessage("TickTimer:" + tickTimer + "lastDispatched:" + lastDispatchedTime + "DIS_DURATION:" + DISPATCH_DURATION, "Space Pirates");
	if(lastDispatchedTime > 0 && lastDispatchedTime + DISPATCH_DURATION <= tickTimer) {
		setAntennaName(NORMAL_ANTENNA_NAME);
		lastDispatchedTime = 0;
	}
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
	cameras = getCameras();
	if(cameras.Count == 0){
		Echo("No camera found.");
	}
	else {
		foreach (IMyCameraBlock camera in cameras) {
			if(!camera.EnableRaycast) {
				camera.EnableRaycast = true;
			}
		}
	}
	long targetId = detectNearbyEnemies();
	if(targetId != 0) {
		Echo("targetId:" + targetId + " response:" + activeTargets[targetId]);
		dispatchResponse(targetId, activeTargets[targetId]);
	}

}

long detectNearbyEnemies() {
	//TODO: probably go for closest or radioactive first
	List<long> enemyGridIds = GetAllEnemyGrids("None", visualRange);
	Echo("Found " + enemyGridIds.Count + " enemy grids.\n");
	short response = 0;
	long target = 0;
	bool NPCGridIdsUpdated = false;
	Echo("Known NPC/Player grid counts:" + knownNPCGridIds.Count + "/" + knownPlayerGridIds.Count);

	foreach (long gridId in enemyGridIds) {
		if(!knownPlayerGridIds.Contains(gridId)) {
			//SendChatMessage("_!known player grid", "Space Pirates");
			if(knownNPCGridIds.Contains(gridId)) {
				//SendChatMessage("__known NPC grid", "Space Pirates");
				continue;
			}
			else {
				if(NPCGridIdsUpdated) {
					//SendChatMessage("___NPCs already updated, adding new player grid", "Space Pirates");
					knownPlayerGridIds.Add(gridId);
				}
				else {
					//SendChatMessage("___Updating NPC grids", "Space Pirates");
					knownNPCGridIds = getNPCGridIds();
					NPCGridIdsUpdated = true;
					string gridsString = "";
					foreach(long id in knownNPCGridIds) {
						gridsString += id + ",";
					}
					//SendChatMessage("updated NPCs:" + gridsString, "Space Pirates");

					if(knownNPCGridIds.Contains(gridId)) {
						//SendChatMessage("____gridId was in newly updated NPC grid list", "Space Pirates");
						continue;
					}
					else {
						//SendChatMessage("____gridId was NOT in newly updated NPC grid list. Adding to know player grids", "Space Pirates");
						knownPlayerGridIds.Add(gridId);
					}
				}
				
			}
		}
		//SendChatMessage("MDEI: " + mdei.Name + " " + mdei.Type, "Space Pirates");
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
				if(!activeTargets.ContainsKey(target)) {
					activeTargets.Add(target, response);
				}
				return target;
			}
		}
	}
	Echo("No valid targets found.");
	return target;
}

HashSet<long> getNPCGridIds() {
	string[] factions = {"MILT", "IMDC",  "CIVL", "Icore", "Junk", "LT-V", "Rust", "VCOR", "Traders"};
	int dist = (int)visualRange + 2000;
	HashSet<long> grids = knownNPCGridIds;
	for(int i = 0; i < factions.Length; ++i) {
		var factionGrids = GetAllEnemyGrids(factions[i], dist);
		//SendChatMessage(factions[i] + ":" + factionGrids.Count, "Space Pirates");
		foreach(long id in factionGrids) {
			if(!grids.Contains(id)) {
				grids.Add(id);
			}
		}
		//SendChatMessage("grids.Count:"+ ":" + grids.Count, "Space Pirates");

	}
	return grids;
}

MyDetectedEntityType castRayAt(long gridId) {
	//TODO - check for empty enum (this will either be null or set to empty)
	Vector3D targetPos = GetTrackedEntityPosition(gridId);
	foreach (IMyCameraBlock camera in cameras) {
		if(camera != null && camera.IsFunctional && camera.CanScan(targetPos)) {
			Echo("Recon camera found");
			Echo("Range: " + camera.AvailableScanRange.ToString());
			MyDetectedEntityInfo mdei = camera.Raycast(targetPos);
			Echo("MDEI:" + mdei);
			return mdei.Type;
		}
	}
	Echo("No viable camera found");

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
		setAntennaName(antennaName);
	}
	lastDispatchedTime = tickTimer;
	//NEED TO CHECK IF THIS CAN SPAWN prefabs with drone behaviour or only prefabs with no behaviour. If the latter, probably need to use antennaes instead
	//SpawnReinforcements(string spawnType, string spawnName, string spawnFaction, bool mustSpawnAll, Vector3D spawnCoords, Vector3D forwardDirection, Vector3D upDirection, Vector3D spawnVelocity)
}

void setAntennaName(String name) {
	List<IMyRadioAntenna> antennae = new List<IMyRadioAntenna>();
	GridTerminalSystem.GetBlocksOfType(antennae);
	if(antennae.Any()) {
		IMyRadioAntenna antenna = antennae[0];
		antenna.CustomName = name;
		antenna.ApplyAction("OnOff_On");
		if(!antenna.GetValue<bool>("EnableBroadCast")) {
			antenna.ApplyAction("EnableBroadCast");
		}
	}
}

List<IMyCameraBlock> getCameras() {
	List<IMyCameraBlock> cameras = new List<IMyCameraBlock>();
	GridTerminalSystem.GetBlocksOfType<IMyCameraBlock>(cameras);
	return cameras;
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

MyDetectedEntityInfo GetMDEI(long entityId){
	
	try{
		
		Me.CustomData = entityId.ToString();
		return Me.GetValue<MyDetectedEntityInfo>("NpcExtender-GetDetectedEntityInfo");
		
	}catch(Exception exc){
		
		Echo("Hard fail NpcExtender-GetDetectedEntityInfo");
		return new MyDetectedEntityInfo();
		
	}
	
}