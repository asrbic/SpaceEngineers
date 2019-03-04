int rank = 1;
double activeAntennaDetectionRange = 0;
double activeReactorDetectionRange = 0;
double visualDetectionRange = 0;
double playerDetectionRange = 0;
double cheatDetectionRange = 0;

/**** INTERNAL - DO NOT MODIFY ****/
int updateFrequency;
int executionFrequency;
double maxDetectionRange;
long tickTimer;
List<IMyRemoteControl> remoteControls = null;
List<IMyCameraBlock> cameras = null;
IMyRemoteControl activeRC = null;
Vector3D currentLocation;

public Program() {
	setExecutionFrequency(50);
	maxDetectionRange = Math.Max(activeAntennaDetectionRange, Math.max(activeReactorDetectionRange, visualDetectionRange));
	remoteControls = getRemoteControls();
	cameras = getCameras();
}

void setExecutionFrequency(int frequency) {
	if(frequency >= 100 && frequency % 100 == 0) {
		Runtime.UpdateFrequency = UpdateFrequency.Update100;
		updateFrequency = 100;
	}
	else if (frequency >= 10 && frequency % 10 == 0) {
		Runtime.UpdateFrequency = UpdateFrequency.Update10;
		updateFrequency = 10;
	}
	else if (frequency >= 1) {
		Runtime.UpdateFrequency = UpdateFrequency.Update1;
		updateFrequency = 1;
	}
	else {
		return;
	}
	executionFrequency = frequency;
	updateFrequency = frequency;
	tickTimer = 0;
}

void Main() {
	if(tickTimer % executionFrequency == 0) {
		run();
	}
	tickTimer += updateFrequency;
}

void run() {
	foreach(IMyRemoteControl rc in remoteControls) {
		if(rc != null && rc.IsFunctional()) {
			activeRC = rc;
			break;
		}
		else {
			remoteControls.Remove(rc);
		}
	}
	if(activeRC == null) {
		Echo("No Remote Control found - aborting.");
		return;
	}
	currentLocation = remoteControl.GetPosition();
}

List<TargetData> getTargets() {
	List<TargetData> targets = new List<TargetData>();
	
	List<long> enemyGridIds = GetAllEnemyGrids("None", maxDetectionRange);
	Echo("Found " + enemyGridIds.Count + " enemy grids.\n");
	bool NPCGridIdsUpdated = false;
	Echo("Known NPC/Player grid counts:" + knownNPCGridIds.Count + "/" + knownPlayerGridIds.Count);

	foreach (long gridId in enemyGridIds) {
		if(isNPCGrid(gridId, knownPlayerGridIds, knownNPCGridIds, NPCGridIdsUpdated)) {
			continue;
		}
		//SendChatMessage("MDEI: " + mdei.Name + " " + mdei.Type, "Space Pirates");
		if(TargetPowered(gridId)) {
			//target grid must be powered to be an eligible target
			Echo("target " + gridId + " is powered");
			Vector3D targetPosition = GetTrackedEntityPosition(gridId);
			int threatLevel;
			long targetGridId;
			long targetBlockId;
			MyDetectedEntityInfo mdei;
			double distanceToTarget = Vector3D.Distance(targetPosition, currentLocation);
			bool detected = false;
			if(inRange(distanceToTarget, cheatDetectionRange)) {
				// cheat detection
				detected = true;
			}
			if(!detected && inRange(distanceToTarget, activeAntennaDetectionRange)) {
				if(TargetIsBroadcasting(gridId)) {
					// antenna detected
					detected = true;
				}
				
			}
			if(!detected && inRange(distanceToTarget, activeReactorDetectionRange)) {
				targetBlockId = findReactors(gridId);
				if(targetBlockId != 0) {
					// radiation detected
					detected = true;
				}
			}
			if(!detected && inRange(distanceToTarget, visualDetectionRange)) {
				mdei = castRayAt(gridId);
				if(mdei.Type == MyDetectedEntityType.SmallGrid || mdei.Type == MyDetectedEntityType.LargeGrid) {
					// visual detection
					detected = true;
					targetBlockId = mdei.EntityId;
				}
			}
			
			if(detected) {
				TargetData td = new TargetData;
				td.position = targetPosition;
				td.gridId = gridId;
				td.blockId = targetBlockId;
				td.mdei = mdei;
			}
		}
	}
	Echo("No valid targets found.");
	return targetId;	
}

bool inRange(double distance, double limit) {
	if(limit > 0 && distance <= limit) {
		return true;
	}
	return false;
}

bool distanceLessThan(Vector3D a, Vector3D b, double dist) {
	return Vector3D.Distance(a, b) < dist;
}

bool isNPCGrid(long gridId, List<long> knownPlayerGridIds, List<long> knownNPCGridIds, bool NPCGridIdsUpdated) {
	if(!knownPlayerGridIds.Contains(gridId)) {
		//SendChatMessage("_!known player grid", "Space Pirates");
		if(knownNPCGridIds.Contains(gridId)) {
			//SendChatMessage("__known NPC grid", "Space Pirates");
			return true;
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
				//foreach(long id in knownNPCGridIds) {
				//	gridsString += id + ",";
				//}
				//SendChatMessage("updated NPCs:" + gridsString, "Space Pirates");

				if(knownNPCGridIds.Contains(gridId)) {
					//SendChatMessage("____gridId was in newly updated NPC grid list", "Space Pirates");
					return true;
				}
				else {
					//SendChatMessage("____gridId was NOT in newly updated NPC grid list. Adding to know player grids", "Space Pirates");
					knownPlayerGridIds.Add(gridId);
				}
			}
			
		}
	}
	return false;
}

MyDetectedEntityInfo castRayAt(long gridId) {
	//TODO - check for empty enum (this will either be null or set to empty)
	foreach(IMyCameraBlock camera in cameras) {
		if(camera != null && camera.IsFunctional()) {
			Echo("Recon camera found");
			Echo("Range: " + camera.AvailableScanRange.ToString());
			Vector3D targetPos = GetTrackedEntityPosition(gridId);
			if(camera.CanScan(targetPos)) {
				MyDetectedEntityInfo mdei = camera.Raycast(targetPos);
				Echo("MDEI:" + mdei);
				return mdei;
			}
			Echo("Cannot cast ray");
		}
		else {
			cameras.Remove(camera);
		}
	}
	return null;
}

List<IMyCameraBlock> getCameras() {
	List<IMyCameraBlock> cameras = new List<IMyCameraBlock>();
	GridTerminalSystem.GetBlocksOfType<IMyCameraBlock>(cameras);
	return cameras;
}

List<IMyRemoteControl> getRemoteControls() {
	List<IMyRemoteControl> remoteControls = new List<IMyRemoteControl>();
	GridTerminalSystem.GetBlocksOfType<IMyRemoteControl>(remoteControls);
	return remoteControls;	
}

HashSet<long> getNPCGridIds(int dist) {
	string[] factions = {"MILT", "IMDC",  "CIVL", "Icore", "Junk", "LT-V", "Rust", "VCOR", "Traders"};
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

struct TargetData {
	Vector3D position;
	int threatLevel;
	long gridId;
	long blockId;
	MyDetectedEntityInfo mdei;
}

/********* MES scripts ********/
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