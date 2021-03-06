double retreatRange = 6000;
double range = 5000;   
double startAltitude = 1200;
double bombardAltitude = 700;
int speedLimit = 50;
int scanProgress = 0;
int scanTime = 0;
long bombardStartTime = 0;
long tickTimer = 0;

const short SPAWNED = 0;
const short FLYING_TO_START = 1;
const short SCANNING = 2;
const short ENEMY_FOUND = 3;
const short FLYING_TO_BOMBARD = 4;
const short BOMBARDING = 5;
const short RETREAT = 6;
const short RETREATING = 7;


const int UPDATE_FREQUENCY = 120;
const int REFILL_FREQUENCY = UPDATE_FREQUENCY * 200;
const int BOMBARD_DURATION = UPDATE_FREQUENCY * 20;
const int SCAN_DURATION = UPDATE_FREQUENCY * 60;

bool planetCentreSet = false;
bool enemyBaseSet = false;
bool dropPointSet = false;
bool bombardSet = false;
short status = SPAWNED;

Vector3D planetCentreGPS;
Vector3D enemyBaseGPS;
Vector3D scanGPS;
Vector3D bombardGPS;
long enemyBaseId;

string lcdName = "LCD Panel";
string remoteControlName = "(MAIN) Remote Control Forward";
string backupRemoteControlName = "(BACKUP) Remote Control Forward";
string ammoCargoName = "Ammo Small Cargo Container";
string fuelCargoName = "Fuel Small Cargo Container";
string bombardLauncherGroupName = "Bombard Launchers";
string bombardGyroName = "Gyroscope Spin";
string forwardMarkerName = "Battery Forwards";
string backwardsMarkerName = "Battery Backwards";
IMyRemoteControl remoteControl = null;	


public Program() {
	Runtime.UpdateFrequency = UpdateFrequency.Update10;
}

void Main() {
	tickTimer += 10;
	if(tickTimer % UPDATE_FREQUENCY == 0) {
		// SendChatMessage("TickTimer:" + tickTimer, "Space Pirates");
		remoteControl = GridTerminalSystem.GetBlockWithName(remoteControlName) as IMyRemoteControl;
		if(remoteControl == null) {
			Echo("no remote control found, aborting");
			return;
		}
		Echo("Remote control found.");
		if(!dropPointSet) {
			Echo("Acquiring planet centre...");
			if(!planetCentreSet) {
				planetCentreSet = getPlanetCentre(ref planetCentreGPS);
			}
			Echo("planet centre: " + planetCentreGPS);
			Echo("Acquiring enemy base location...");
			enemyBaseSet = getPositionOfLargestNearbyEnemyBase(ref enemyBaseGPS);
			Echo("Enemy base location: " + enemyBaseGPS);
			if (planetCentreSet && enemyBaseSet) {
				Echo("Calculating start location...");
				dropPointSet = interpolateByDistance(enemyBaseGPS, planetCentreGPS, startAltitude, ref scanGPS);
				Echo("Start location:" + scanGPS);
				bombardSet = interpolateByDistance(enemyBaseGPS, planetCentreGPS, bombardAltitude, ref bombardGPS);
				Echo("Bombard location:" + bombardGPS);
			}
		}
		else {
			if(status == RETREATING) {
				//despawn if 6km+ away
				//SendChatMessage("RETREATING", "Space Pirates");
				if(Vector3D.Distance(remoteControl.GetPosition(), enemyBaseGPS) > retreatRange) {
					//SendChatMessage("Enemy Base sufficiently far away", "Space Pirates");
					Vector3D nearestPlayer = new Vector3D(0,0,0);
					bool playerExists = remoteControl.GetNearestPlayer(out nearestPlayer);
					if(!playerExists || Vector3D.Distance(remoteControl.GetPosition(), nearestPlayer) > retreatRange) {
						//SendChatMessage("Attempting Despawn", "Space Pirates");
						AttemptDespawn();
					}
				}
			}
			else if(status == RETREAT) {
				Vector3D retreatGPS = new Vector3D(0,0,0);
				IMyTerminalBlock forwardMarker = GridTerminalSystem.GetBlockWithName(forwardMarkerName);
				if(forwardMarker == null) {
					Echo("Nav error, forward reference block not found.");
					return;
				}
				bool gotRetreatLocation = interpolateByDistance(remoteControl.GetPosition(), forwardMarker.GetPosition(), 10000, ref retreatGPS);
				flyToLocation(retreatGPS);
				//SendChatMessage("Retreat GPS:" + retreatGPS, "Space Pirates");
				var lcd = GridTerminalSystem.GetBlockWithName(lcdName) as IMyTextPanel; 
				lcd.WritePublicText(retreatGPS.ToString()); 
				lcd.ShowPublicTextOnScreen();
				SendChatMessage("Returning to base.", "Space Pirates");
				status = RETREATING;
			}
			else if(status == BOMBARDING) {
				if((tickTimer - bombardStartTime) > BOMBARD_DURATION) {
					//SendChatMessage("bombardStartTime:" + bombardStartTime, "Space Pirates");
					toggleBombardment(false);
					status = RETREAT;
					SendChatMessage("Extermination procedure completed.", "Space Pirates");
				}
			}
			else if(status == FLYING_TO_BOMBARD) {
				if(distanceLessThan(remoteControl.GetPosition(), bombardGPS, 20)) {
					status = BOMBARDING;
					toggleBombardment(true);
					bombardStartTime = tickTimer;
				}
			}
			else if(status == SCANNING) {
				if(TargetPowered(enemyBaseId)) {
					scanProgress += 5;
					if(scanProgress % 20 == 0) {
					SendChatMessage("Scanning progress:" + scanProgress + "%", "Space Pirates");
					}
				}
				scanTime += UPDATE_FREQUENCY;
				
				if(scanProgress >= 100) {
					if(bombardSet) {
						flyToLocation(bombardGPS);
						status = FLYING_TO_BOMBARD;
						SendChatMessage("Active enemy installation detected. Exterminating.", "Space Pirates");
					}
					else {
						status = RETREAT;
					}
				}
				else if(scanTime >= SCAN_DURATION) {
					SendChatMessage("No activity detected.", "Space Pirates");
					status = RETREAT;					
				}
			}
			else if(status == FLYING_TO_START) {
				if(distanceLessThan(remoteControl.GetPosition(), scanGPS, 20)) {
					Echo("Start position reached!");
					status = SCANNING;
					SendChatMessage("Scanning for energy signatures...", "Space Pirates");
				}
			}
			else {
				Echo("flying to start location");
				SendChatMessage("Activity detected. Moving to scanning position.", "Space Pirates");
				flyToLocation(scanGPS);
				status = FLYING_TO_START;
			}
		}
		refillIfApplicable();
		printStatus();
	}
}

void toggleBombardment(bool onOff) {
	List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
	IMyBlockGroup group = GridTerminalSystem.GetBlockGroupWithName (bombardLauncherGroupName);
	group.GetBlocks(blocks);
	foreach (IMyTerminalBlock block in blocks) {
		block.SetValueBool("Shoot", onOff);
	}
	IMyTerminalBlock gyro = GridTerminalSystem.GetBlockWithName(bombardGyroName);
	gyro.SetValueBool("Override", onOff);
}

void refillIfApplicable() {
	if(tickTimer % REFILL_FREQUENCY == 0) {
		FillGasGenerators();
		IMyTerminalBlock fuelCargo = GridTerminalSystem.GetBlockWithName(fuelCargoName) as IMyTerminalBlock;
		bool createdSuccessfully = CreateItemInInventory("MyObjectBuilder_Ingot", "Uranium", 20, fuelCargo.GetId());
		if(!createdSuccessfully) {
			Echo("Failed to refill reactor fuel");
		}
	}
}

void printStatus() {
	Echo("STATUS: " + status);
	Echo("planetCentreSet:" + planetCentreSet);
	Echo("enemyBaseSet:" + enemyBaseSet);
	Echo("dropPointSet:" + dropPointSet);
	Echo("bombardSet:" + bombardSet);
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

bool getPositionOfLargestNearbyEnemyBase(ref Vector3D coords) {
	int biggestGridSize = -1;
	long biggestGridId = -1;
	List<long> enemyGridIds = GetAllEnemyGrids("None", range);
	Echo("Found " + enemyGridIds.Count + " enemy grids.\n");
	foreach (long gridId in enemyGridIds) {
		bool isStatic = TargetIsStatic(gridId);
		bool isPowered = TargetPowered(gridId);
		if(isStatic && isPowered) {
			int gridSize = TargetBlockCount(gridId);
			Echo("Grid " + gridId + " is static, powered and has " + gridSize + " blocks\n");
			if(gridSize > biggestGridSize) {
				biggestGridId = gridId;
				biggestGridSize = gridSize;
			}
		}
	}
	
	if(biggestGridId > -1) {
		enemyBaseId = biggestGridId;
		coords = GetTrackedEntityPosition(biggestGridId);
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

bool FillGasGenerators() {
	
	try{
		
		return Me.GetValue<bool>("NpcExtender-IceRefill");
		
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