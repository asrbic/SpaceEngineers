#region header
using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Text;
using System;
using VRage.Collections;
using VRage.Game.Components;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRage.Game;
using VRageMath;
using VRage.Library;
using VRage;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        #endregion
        //General settings
        int rank = 1;
        Tactic tactic = Tactic.Default;
        double range = 500;

        //tactic specific settings
        CarrierSpawn[] spawns = { new CarrierSpawn{ } };
        string[] carrierPrefabs = { "[PvEThreat] Seagull", "[PvEThreat] Minor" };
        //Vector3D[] carrierPrefabOffsets = 
        
        //detection settings
        double activeAntennaDetectionRange = 0;
        double activeReactorDetectionRange = 0;
        double visualDetectionRange = 0;
        double playerDetectionRange = 0;
        double cheatDetectionRange = 0;

        /**** INTERNAL - DO NOT MODIFY BELOW HERE ****/

        public enum Tactic
        {
            Default, Small, Large, Carrier, Boss, GravityBombard, Ram, MissileSniper, StaticSniper
        };

        struct TargetData
        {
            public Vector3D position;
            public int threatLevel;
            public long gridId;
            public long blockId;
            public MyDetectedEntityInfo mdei;
        }

        struct AllyData
        {
            public long addr;
            public long gridId;
            public long lastUpdatedTick;
            public int rank;
            public double range;
            public MyDetectedEntityInfo mdei;
            public Tactic tactic;
            public Command cmd;
        }

        struct Command
        {
            public CommandType type;
            public long targetId;
            public Vector3D targetVec;
            public Status status;
        }

        public enum CommandType
        {
            None, Info, Move, Formation, FormationLead, Attack, Bombard
        };

        public enum Status
        {
            New, Started, Finished
        };

        struct CarrierSpawn
        {
            string[] subTypeIds;
            Vector3D[] offsets;
            int supply;

            CarrierSpawn(string[] subTypeIds, Vector3D[] offsets, int supply)
            {
                this.subTypeIds = subTypeIds;
                this.offsets = offsets;
                this.supply = supply;
            }
        };


        int updateFrequency;
        int executionFrequency;
        long tickTimer;
        bool execute;
        long lastAllyScan = 0;
        long gridId;
        long commanderGridId;

        List<IMyRemoteControl> remoteControls = null;
        List<IMyRadioAntenna> antennae = null;
        List<IMyCameraBlock> cameras = null;
        IMyRemoteControl activeRC = null;
        Vector3D currentLocation;

        Command myCommand;
        double MOVE_WAYPOINT_TOLERANCE = 50;
        double FORMATION_WAYPOINT_TOLERANCE = 20;

        double maxDetectionRange;
        HashSet<long> knownNPCGridIds = new HashSet<long>();
        HashSet<long> knownPlayerGridIds = new HashSet<long>();
        const double NPC_RANGE_ADD = 1000;

        const double ALLY_RANGE_MULT = 1000;
        const long ALLY_REFRESH_FREQUENCY = 1000;
        bool isCommander;
        IMyBroadcastListener handshakeListener;

        const string COMM_HANDSHAKE = "HANDSHAKE";
        const string COMM_STATUS = "STATUS";
        const string COMM_COMMAND = "CMD";

        const string FORMATION_PB_NAME = "FormationPB";
        IMyProgrammableBlock formationPB;
        int formationStatus = 0;

        Dictionary<long, TargetData> knownTargets = new Dictionary<long, TargetData>();
        Dictionary<long, AllyData> knownAllies = new Dictionary<long, AllyData>();

        public Program()
        {
            gridId = Me.CubeGrid.EntityId;
            handshakeListener = IGC.RegisterBroadcastListener(COMM_HANDSHAKE);
            handshakeListener.SetMessageCallback(COMM_HANDSHAKE);
            isCommander = true;
            myCommand = new Command();
            myCommand.type = CommandType.None;
            myCommand.status = Status.New;
            setExecutionFrequency(50);
            maxDetectionRange = Math.Max(activeAntennaDetectionRange, max(activeReactorDetectionRange, visualDetectionRange));
            remoteControls = new List<IMyRemoteControl>();
            GridTerminalSystem.GetBlocksOfType<IMyRemoteControl>(remoteControls);
            cameras = new List<IMyCameraBlock>();
            GridTerminalSystem.GetBlocksOfType<IMyCameraBlock>(cameras);
            antennae = new List<IMyRadioAntenna>();
            GridTerminalSystem.GetBlocksOfType<IMyRadioAntenna>(antennae);
            formationPB = (IMyProgrammableBlock)GridTerminalSystem.GetBlockGroupWithName(FORMATION_PB_NAME);
        }

        void setExecutionFrequency(int frequency)
        {
            if (frequency >= 100 && frequency % 100 == 0)
            {
                Runtime.UpdateFrequency = UpdateFrequency.Update100;
                updateFrequency = 100;
            }
            else if (frequency >= 10 && frequency % 10 == 0)
            {
                Runtime.UpdateFrequency = UpdateFrequency.Update10;
                updateFrequency = 10;
            }
            else if (frequency >= 1)
            {
                Runtime.UpdateFrequency = UpdateFrequency.Update1;
                updateFrequency = 1;
            }
            else
            {
                Runtime.UpdateFrequency = UpdateFrequency.None;
                updateFrequency = 0;
                return;
            }
            executionFrequency = frequency;
            tickTimer = 0;
            execute = true;
        }

        void Main(string argument, UpdateType updateSource)
        {
            //if (argument != null && updateSource == UpdateType.IGC)
            //{
            //}
            if (execute)
            {
                if (tickTimer % executionFrequency == 0)
                {
                    run();
                }
            }
            tickTimer += updateFrequency;
        }

        void run()
        {
            if (!setup())
            {
                return;
            }
            foreach (AllyData ad in knownAllies.Values)
            {
                Echo("ally info| isCommander:" + (commanderGridId == ad.gridId) + " id:" + ad.gridId + " rank:" + ad.rank + "\n");
            }
            handleMessages();
            if (isCommander)
            {
                updateTargets();
                if (knownTargets.Any())
                {
                    updateAllies();
                }
            }
            executeCommand();
        }

        bool setup()
        {
            if (activeRC == null)
            {

                foreach (IMyRemoteControl rc in remoteControls)
                {
                    if (rc != null && rc.IsFunctional)
                    {
                        activeRC = rc;
                        break;
                    }
                    else
                    {
                        remoteControls.Remove(rc);
                    }
                }
            }
            if (activeRC == null)
            {
                Echo("No Remote Control found - aborting.");
                return false;
            }
            currentLocation = activeRC.GetPosition();
            return true;
        }

        /******************* COMMS *************************/
        void handleMessages()
        {
            /* valid types to send
             *  Primitives
                String
                MyTuple of safe types
                Ton of game math structs like all vectors, bounding boxes, matrices, quaternions, ...
                Immutable collections of safe types (not builders)
                ImmutableArray
                ImmutableList
                ImmutableQueue
                ImmutableStack
                ImmutableHashSet
                ImmutableSortedSet
                ImmutableDictionary
                ImmutableSortedDictionary*/

            handleHandshakes();
            while (IGC.UnicastListener.HasPendingMessage)
            {
                MyIGCMessage msg = IGC.UnicastListener.AcceptMessage();
                if (msg.Tag.Equals(COMM_HANDSHAKE))
                {
                    handleHandshakeMessage(msg, false);
                }
                else if (msg.Tag.Equals(COMM_COMMAND))
                {
                    handleCommandMessage(msg);
                }
                else if (msg.Tag.Equals(COMM_STATUS))
                {

                }
            }
        }


        void handleHandshakes()
        {
            while (handshakeListener.HasPendingMessage)
            {
                SendChatMessage("Broadcast message recieved", "SPRT");
                MyIGCMessage msg = handshakeListener.AcceptMessage();
                handleHandshakeMessage(msg, true);
            }
        }


        void handleCommandMessage(MyIGCMessage msg)
        {
            Command cmd = deserialiseCommand(msg);
            if(isCommander)
            {//probably target data from previous commander
                if(cmd.type == CommandType.Info)
                {
                    if(!knownTargets.ContainsKey(cmd.targetId))
                    {
                        TargetData td = new TargetData();
                        td.gridId = cmd.targetId;
                        td.position = cmd.targetVec;
                        knownTargets[cmd.targetId] = td;
                    }
                }
            }
            else
            {//command directive
                myCommand = cmd;
                setRunSpeed(cmd.type);
                if(myCommand.type == CommandType.Formation)
                {
                    formationStatus = 0;
                }
            }
        }

        void setRunSpeed(CommandType ct)
        {
            int newExecutionFrequency = executionFrequency;

            if (ct == CommandType.Move)
            {
                newExecutionFrequency = 50;
            }
            else if (ct == CommandType.None)
            {
                newExecutionFrequency = 200;
            }
            else if(ct == CommandType.Formation)
            {
                newExecutionFrequency = 5;
            }
            else if (ct == CommandType.Attack)
            {
                newExecutionFrequency = 1;
            }
            else if(ct == CommandType.Bombard)
            {
                newExecutionFrequency = 10;
            }

            if (executionFrequency != newExecutionFrequency)
            {
                setExecutionFrequency(executionFrequency);
            }
        }

        void handleHandshakeMessage(MyIGCMessage msg, bool reply)
        {
            try
            {
                //sourceCommId, sourceGridId, rank, 
                MyTuple<long, long, int, int, double> tuple = getHandshakeMsgData(msg);
                AllyData ad;
                long sourceCommId = tuple.Item1;
                long sourceGridId = tuple.Item2;
                int sourceRank = tuple.Item3;
                Tactic sourceTactic = (Tactic)tuple.Item4;
                double sourceRange = tuple.Item5;
                SendChatMessage("Handling handshake message: tag:"+ msg.Tag + " source:" + sourceGridId + " rank:" + sourceRank + " tactic:" + sourceTactic.ToString() + " range:" + sourceRange, "SPRT");
                if (knownAllies.ContainsKey(sourceGridId))
                {
                    ad = knownAllies[sourceGridId];
                }
                else
                {
                    ad = new AllyData();
                    ad.gridId = sourceGridId;
                }
                ad.rank = sourceRank;
                ad.tactic = sourceTactic;
                ad.addr = sourceCommId;
                ad.range = sourceRange;
                knownAllies[sourceGridId] = ad;
                if ((sourceRank > rank || (sourceRank == rank && sourceGridId < gridId)))
                {//pass baton
                    if (isCommander)
                    {
                        isCommander = false;
                        if(reply)
                        {
                            foreach (TargetData td in knownTargets.Values)
                            {// update new commander with my active targets
                                IGC.SendUnicastMessage(sourceCommId, COMM_COMMAND, targetDataToCommand(td));
                            }
                        }
                    }
                    commanderGridId = sourceGridId;
                }
                if(reply)
                {
                    IGC.SendUnicastMessage(sourceCommId, COMM_HANDSHAKE, getHandshakeMsgData());
                    
                }
                
            }
            catch
            {

            }
        }

        MyTuple<long, long, int, int, double> getHandshakeMsgData(MyIGCMessage msg)
        {
            return (MyTuple<long, long, int, int, double>)msg.Data;
        }

        MyTuple<long, long, int, int, double> getHandshakeMsgData()
        {
            return new MyTuple<long, long, int, int, double>(IGC.Me, gridId, rank, (int)tactic, range);
        }

        bool sendCommand(long targetAddr, Command cmd)
        {
            if(IGC.IsEndpointReachable(targetAddr))
            {
                IGC.SendUnicastMessage(targetAddr, COMM_COMMAND, serialiseCommand(cmd));
                return true;
            }
            else
            {
                return false;
            }
        }

        MyTuple<int, long, Vector3D> serialiseCommand(Command cmd)
        {
            return new MyTuple<int, long, Vector3D>((int)cmd.type, cmd.targetId, cmd.targetVec);
        }

        Command deserialiseCommand(MyIGCMessage msg)
        {
            MyTuple<int, long, Vector3D> tuple = (MyTuple<int, long, Vector3D>)msg.Data;
            Command cmd = new Command();
            cmd.type = (CommandType)tuple.Item1;
            cmd.targetId = tuple.Item2;
            cmd.targetVec = tuple.Item3;
            cmd.status = Status.New;
            return cmd;
        }

        MyTuple<int, long, Vector3D> targetDataToCommand(TargetData td)
        {
            return new MyTuple<int, long, Vector3D>((int)CommandType.Info, td.gridId, td.position);
        }

        /******************* CORE LOOPS *************************/
        void executeCommand()
        {//TODO
            if(formationStatus != 0 && myCommand.type != CommandType.Formation)
            {// attempt to turn off formation PB
                if (formationPB != null && formationPB.IsWorking)
                {
                    if(formationPB.TryRun("start"))
                    {
                        formationStatus = 0;
                    }
                }
                else
                {
                    formationStatus = 0;
                }
            }
            if (myCommand.status == Status.Finished)
            {
                return;
            }
            Vector3D grav = activeRC.GetNaturalGravity();
            bool inGravity = false;
            if (grav.Length() > 0)
            {
                inGravity = true;
            }
            CommandType ct = myCommand.type;
            if (ct == CommandType.None)
            {// Do nothing

            }
            else if (ct == CommandType.Move || ct == CommandType.FormationLead)
            {// Move to target position
                if (myCommand.status == Status.New)
                {//initiate movement
                    flyToLocation(myCommand.targetVec);
                    myCommand.status = Status.Started;
                }
                else if (myCommand.status == Status.Started)
                {// Check if end condition met
                    if (distanceLessThan(currentLocation, myCommand.targetVec, MOVE_WAYPOINT_TOLERANCE))
                    {//End condition met - inform commander
                        notifyCommandCompleted();
                    }
                }
                if (ct == CommandType.FormationLead)
                {//I am the leader
                    IGC.SendBroadcastMessage<MyTuple<MatrixD, Vector3D>>("FSLeaderSystem1", 
                        new MyTuple<MatrixD, Vector3D>(activeRC.WorldMatrix, activeRC.GetShipVelocities().LinearVelocity));
                }
            }
            else if (ct == CommandType.Formation)
            {// Move into formation with target grid with offset dictated by vec
                if(myCommand.status == Status.New)
                {//Activate formation PB
                    if (formationPB != null && formationPB.IsWorking)
                    {
                        if (formationStatus == 0)
                        {//run success
                            if (formationPB.TryRun("setoffset;" + myCommand.targetVec.X + ";" + myCommand.targetVec.Y + ";" + myCommand.targetVec.Z))
                                formationStatus = 1;
                        }
                        if (formationStatus == 1)
                        {
                            if (formationPB.TryRun("start"))
                            {//run success
                                formationStatus = 2;
                                myCommand.status = Status.Started;
                            }
                        }
                    }
                }
            }
            else if (ct == CommandType.Bombard)
            {// attack the target from a distance with tactic x
                long targetId = myCommand.targetId;
                //probably notify complete command if ammunition/drones exhausted
                if(tactic == Tactic.Carrier)
                {//spawn drones etc
                    
                }
                else if(tactic == Tactic.GravityBombard)
                {// move to position above target and bombard

                }
                else if(tactic == Tactic.MissileSniper)
                {// move to position at range then fire missiles

                }
                else if(tactic == Tactic.StaticSniper)
                {// move to position at range then fire static weapons (if any)
                    
                }
            }
            else if(ct == CommandType.Attack)
            {// engage target with tactic x

            }

        }

        void enterFormation(long targetId, Vector3D offset)
        {//woefully incomplete
            Vector3D leadPos = GetTrackedEntityPosition(targetId);
            leadPos += offset;
            flyToLocation(leadPos);
        }

        void updateAllies()
        {
            scanForNewAllies();
            updateCommands();
        }

        void updateCommands()
        {
            if (knownTargets.Any())
            {

            }
            else
            {//form up on commander if no targets
                foreach(long id in knownAllies.Keys)
                {
                    AllyData ad = knownAllies[id];
                    if(ad.cmd.type != CommandType.Formation)
                    {
                        ad.cmd = getFormationCommand(ad, gridId);
                        knownAllies[id] = ad;
                        sendCommand(ad.addr, ad.cmd);
                    }
                }
            }
        }

        Command getFormationCommand(AllyData ad, long target)
        {
            Command cmd = new Command
            {
                type = CommandType.Formation,
                status = Status.New,
                targetId = target,
                //need to amend this
                targetVec = new Vector3D(rank * 100, 0, 0)
            };
            return cmd;
        }

        void notifyCommandCompleted()
        {
            myCommand.status = Status.Finished;
            sendCommand(knownAllies[commanderGridId].addr, myCommand);
        }


        void scanForNewAllies()
        {
            if (tickTimer - lastAllyScan >= ALLY_REFRESH_FREQUENCY)
            {
                lastAllyScan = tickTimer;
                List<long> alliedGrids = GetAllAlliedGrids(rank * ALLY_RANGE_MULT);
                foreach (long allyGridId in alliedGrids)
                {
                    if (!knownAllies.ContainsKey(allyGridId))
                    {
                        AllyData ad = new AllyData();
                        ad.gridId = allyGridId;
                        ad.rank = -1;
                        knownAllies.Add(allyGridId, ad);
                        IGC.SendBroadcastMessage(COMM_HANDSHAKE, getHandshakeMsgData());
                    }
                }
            }
        }

        void updateTargets()
        {
            List<long> enemyGridIds = GetAllEnemyGrids("None", maxDetectionRange);
            Echo("Found " + enemyGridIds.Count + " enemy grids.\n");
            bool NPCGridIdsUpdated = false;
            Echo("Known NPC/Player grid counts:" + knownNPCGridIds.Count + "/" + knownPlayerGridIds.Count);

            foreach (long gridId in enemyGridIds)
            {
                if (isNPCGrid(gridId, NPCGridIdsUpdated))
                {
                    continue;
                }
                //SendChatMessage("MDEI: " + mdei.Name + " " + mdei.Type, "Space Pirates");
                if (TargetPowered(gridId))
                {
                    //target grid must be powered to be an eligible target
                    Echo("target " + gridId + " is powered");
                    Vector3D targetPosition = GetTrackedEntityPosition(gridId);
                    int threatLevel = 0;
                    long targetBlockId = 0;
                    MyDetectedEntityInfo mdei = new MyDetectedEntityInfo();
                    double distanceToTarget = Vector3D.Distance(targetPosition, currentLocation);
                    bool detected = false;
                    if (inRange(distanceToTarget, cheatDetectionRange))
                    {
                        // cheat detection
                        detected = true;
                    }
                    if (!detected && inRange(distanceToTarget, activeAntennaDetectionRange))
                    {
                        if (TargetIsBroadcasting(gridId))
                        {
                            // antenna detected
                            detected = true;
                        }

                    }
                    if (!detected && inRange(distanceToTarget, activeReactorDetectionRange))
                    {
                        targetBlockId = findReactors(gridId);
                        if (targetBlockId != 0)
                        {
                            // radiation detected
                            mdei = GetMDEI(targetBlockId);
                            detected = true;
                        }
                    }
                    if (!detected && inRange(distanceToTarget, visualDetectionRange))
                    {
                        mdei = castRayAt(gridId);
                        if (!mdei.IsEmpty() && (mdei.Type == MyDetectedEntityType.SmallGrid || mdei.Type == MyDetectedEntityType.LargeGrid))
                        {
                            // visual detection
                            detected = true;
                            targetBlockId = mdei.EntityId;
                            targetPosition = mdei.HitPosition.Value;
                        }
                    }

                    if (detected)
                    {
                        TargetData td;
                        if (knownTargets.ContainsKey(gridId))
                        {
                            knownTargets.TryGetValue(gridId, out td);
                        }
                        else
                        {
                            td = new TargetData();
                            td.gridId = gridId;
                            knownTargets.Add(gridId, td);
                        }
                        if (mdei.IsEmpty())
                        {
                            mdei = GetMDEI(gridId);
                        }
                        td.position = targetPosition;
                        td.blockId = targetBlockId;
                        td.mdei = mdei;
                        td.threatLevel = threatLevel;
                    }
                }
            }
            Echo("No valid targets found.");
        }

        void flyToLocation(Vector3D coords, float speedLimit = 20, bool collisionAvoidance = true)
        {
            activeRC.ClearWaypoints();
            activeRC.AddWaypoint(coords, "1");
            activeRC.FlightMode = FlightMode.OneWay;
            activeRC.IsMainCockpit = true;
            activeRC.SetAutoPilotEnabled(true);
            activeRC.SetCollisionAvoidance(collisionAvoidance);
            activeRC.SpeedLimit = speedLimit;
        }

        bool inRange(double distance, double limit)
        {
            if (limit > 0 && distance <= limit)
            {
                return true;
            }
            return false;
        }

        long findReactors(long gridId)
        {
            long blockId = GetTargetShipSystem(gridId, "MyObjectBuilder_Reactor");
            return blockId;
        }

        bool distanceLessThan(Vector3D a, Vector3D b, double dist)
        {
            return Vector3D.Distance(a, b) < dist;
        }

        bool isNPCGrid(long gridId, bool NPCGridIdsUpdated)
        {
            if (!knownPlayerGridIds.Contains(gridId))
            {
                //SendChatMessage("_!known player grid", "Space Pirates");
                if (knownNPCGridIds.Contains(gridId))
                {
                    //SendChatMessage("__known NPC grid", "Space Pirates");
                    return true;
                }
                else
                {
                    if (NPCGridIdsUpdated)
                    {
                        //SendChatMessage("___NPCs already updated, adding new player grid", "Space Pirates");
                        knownPlayerGridIds.Add(gridId);
                    }
                    else
                    {
                        //SendChatMessage("___Updating NPC grids", "Space Pirates");
                        updateNPCGridIds();
                        NPCGridIdsUpdated = true;
                        //string gridsString = "";
                        //foreach(long id in knownNPCGridIds) {
                        //	gridsString += id + ",";
                        //}
                        //SendChatMessage("updated NPCs:" + gridsString, "Space Pirates");

                        if (knownNPCGridIds.Contains(gridId))
                        {
                            //SendChatMessage("____gridId was in newly updated NPC grid list", "Space Pirates");
                            return true;
                        }
                        else
                        {
                            //SendChatMessage("____gridId was NOT in newly updated NPC grid list. Adding to know player grids", "Space Pirates");
                            knownPlayerGridIds.Add(gridId);
                        }
                    }

                }
            }
            return false;
        }

        MyDetectedEntityInfo castRayAt(long gridId)
        {
            //TODO - check for empty enum (this will either be null or set to empty)
            foreach (IMyCameraBlock camera in cameras)
            {
                if (camera != null && camera.IsFunctional)
                {
                    Echo("Recon camera found");
                    Echo("Range: " + camera.AvailableScanRange.ToString());
                    Vector3D targetPos = GetTrackedEntityPosition(gridId);
                    if (camera.CanScan(targetPos))
                    {
                        MyDetectedEntityInfo mdei = camera.Raycast(targetPos);
                        Echo("MDEI:" + mdei);
                        return mdei;
                    }
                    Echo("Cannot cast ray");
                }
                else
                {
                    cameras.Remove(camera);
                }
            }
            return new MyDetectedEntityInfo();
        }

        void updateNPCGridIds()
        {
            string[] factions = { "MILT", "IMDC", "CIVL", "Icore", "Junk", "LT-V", "Rust", "VCOR", "Traders" };
            HashSet<long> grids = knownNPCGridIds;
            for (int i = 0; i < factions.Length; ++i)
            {
                var factionGrids = GetAllEnemyGrids(factions[i], maxDetectionRange + NPC_RANGE_ADD);
                //SendChatMessage(factions[i] + ":" + factionGrids.Count, "Space Pirates");
                foreach (long id in factionGrids)
                {
                    if (!grids.Contains(id))
                    {
                        grids.Add(id);
                    }
                }
                //SendChatMessage("grids.Count:"+ ":" + grids.Count, "Space Pirates");

            }
        }

        string getActionsForClass()
        {
            List<ITerminalProperty> a = new List<ITerminalProperty>();
            string s = "";
            Me.GetProperties(a);
            foreach (ITerminalProperty ac in a)
                s += ac.Id + " ";
            SendChatMessage("Me.actions:" + s, "SPRT");
            return s;
        }

        /********* MES scripts ********/
        List<long> GetAllEnemyGrids(string specificFaction = "None", double distanceToCheck = 15000)
        {

            try
            {

                Me.CustomData = specificFaction + "\n" + distanceToCheck.ToString();
                return Me.GetValue<List<long>>("NpcExtender-GetAllEnemies");

            }
            catch (Exception exc)
            {

                Echo("NpcExtender-GetAllEnemy Hard Fail");
                return new List<long>();

            }

        }

        bool TargetIsStatic(long entityId)
        {

            try
            {

                Me.CustomData = entityId.ToString();
                return Me.GetValue<bool>("NpcExtender-TargetIsStatic");

            }
            catch (Exception exc)
            {

                return false;

            }

        }

        bool TargetPowered(long entityId)
        {

            try
            {

                Me.CustomData = entityId.ToString();
                return Me.GetValue<bool>("NpcExtender-TargetPowered");

            }
            catch (Exception exc)
            {

                return false;

            }

        }

        int TargetBlockCount(long entityId)
        {

            try
            {

                Me.CustomData = entityId.ToString();
                return Me.GetValue<int>("NpcExtender-TargetBlockCount");

            }
            catch (Exception exc)
            {

                return 0;

            }

        }

        Vector3D GetTrackedEntityPosition(long entityId)
        {

            try
            {

                Me.CustomData = entityId.ToString();
                return Me.GetValue<Vector3D>("NpcExtender-TrackEntity");

            }
            catch (Exception exc)
            {

                return new Vector3D(0, 0, 0);

            }

        }

        bool SendChatMessage(string message, string author, string audioClip = "")
        {

            double broadcastDistance = 0;
            var antennaList = new List<IMyRadioAntenna>();
            GridTerminalSystem.GetBlocksOfType<IMyRadioAntenna>(antennaList);

            foreach (var antenna in antennaList)
            {

                if (antenna.IsFunctional == false || antenna.Enabled == false || antenna.EnableBroadcasting == false)
                {

                    continue;

                }

                var antennaRange = (double)antenna.Radius;

                if (antennaRange > broadcastDistance)
                {

                    broadcastDistance = antennaRange;

                }

            }

            if (broadcastDistance == 0)
            {

                return false;

            }

            try
            {

                string sendData = message + "\n" + author + "\n" + broadcastDistance.ToString() + "\n" + audioClip;
                Me.CustomData = sendData;
                return Me.GetValue<bool>("NpcExtender-ChatToPlayers");

            }
            catch (Exception exc)
            {

                return false;

            }

        }

        bool CreateItemInInventory(string type, string subtype, float amount, long blockEntityId)
        {

            Me.CustomData = type + "\n" + subtype + "\n" + amount.ToString() + "\n" + blockEntityId.ToString();

            try
            {

                return Me.GetValue<bool>("NpcExtender-CreateItemInInventory");

            }
            catch (Exception exc)
            {

                return false;

            }

        }

        bool AttemptDespawn()
        {

            try
            {

                return Me.GetValue<bool>("NpcExtender-DespawnDrone");

            }
            catch (Exception exc)
            {

                return false;

            }

        }

        bool SpawnReinforcements(string spawnType, string spawnName, string spawnFaction, bool mustSpawnAll, Vector3D spawnCoords, Vector3D forwardDirection, Vector3D upDirection, Vector3D spawnVelocity)
        {

            string spawnData = spawnType + "\n";
            spawnData += spawnName + "\n";
            spawnData += spawnFaction + "\n";
            spawnData += mustSpawnAll.ToString() + "\n";
            spawnData += spawnCoords.ToString() + "\n";
            spawnData += forwardDirection.ToString() + "\n";
            spawnData += upDirection.ToString() + "\n";
            spawnData += spawnVelocity.ToString();

            try
            {

                Me.CustomData = spawnData;
                return Me.GetValue<bool>("NpcExtender-SpawnReinforcements");

            }
            catch (Exception exc)
            {

                return false;

            }

        }

        bool TargetCanFly(long entityId)
        {

            try
            {

                Me.CustomData = entityId.ToString();
                return Me.GetValue<bool>("NpcExtender-TargetCanFly");

            }
            catch (Exception exc)
            {

                return false;

            }

        }

        bool TargetIsBroadcasting(long entityId, bool checkAntennas = true, bool checkBeacons = true)
        {

            try
            {

                Me.CustomData = entityId.ToString() + "\n" + checkAntennas.ToString() + "\n" + checkBeacons.ToString();
                return Me.GetValue<bool>("NpcExtender-TargetIsBroadcasting");

            }
            catch (Exception exc)
            {

                return false;

            }

        }

        Vector3D GetNearestPlanetPosition(Vector3D checkCoords)
        {

            try
            {

                Me.CustomData = checkCoords.ToString();
                return Me.GetValue<Vector3D>("NpcExtender-GetNearestPlanetPosition");

            }
            catch (Exception exc)
            {

                return Vector3D.Zero;

            }

        }

        long GetTargetShipSystem(long targetEntityId, string targetSystems = "MyObjectBuilder_Reactor")
        {
            try
            {

                Me.CustomData = targetEntityId.ToString() + "\n" + targetSystems;
                return Me.GetValue<long>("NpcExtender-GetTargetShipSystem");

            }
            catch (Exception exc)
            {

                return 0;

            }

        }

        double max(double a, double b)
        {
            if (a > b)
            {
                return a;
            }
            return b;
        }

        MyDetectedEntityInfo GetMDEI(long entityId)
        {

            try
            {

                Me.CustomData = entityId.ToString();
                return Me.GetValue<MyDetectedEntityInfo>("NpcExtender-GetDetectedEntityInfo");

            }
            catch (Exception exc)
            {

                Echo("Hard fail NpcExtender-GetDetectedEntityInfo");
                return new MyDetectedEntityInfo();

            }

        }

        List<long> GetAllAlliedGrids(double distanceToCheck = 15000)
        {

            try
            {

                Me.CustomData = distanceToCheck.ToString();
                return Me.GetValue<List<long>>("NpcExtender-GetAllAllies");

            }
            catch (Exception exc)
            {

                return new List<long>();

            }

        }
        #region footer
        /////////////////////////////////
    }
}
#endregion