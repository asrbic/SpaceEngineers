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
        #region public
        //General settings
        AllyData myData = new AllyData
        {
            rank = 1,
            tactic = Tactic.Default,
            range = 500
        };
        //tactic specific settings
        //Carrier
        PrefabData[] spawns = { new PrefabData("[PvEThreat] Seagull", 1), new PrefabData("[PvEThreat] Minor", 2), new PrefabData("[PvEThreat] Ibis", 2), new PrefabData("[PvEThreat] Magpie", 3) };
        Vector3D[] prefabOffsets = { new Vector3D(20, 0, 0), new Vector3D(40, 0, 0), new Vector3D(60, 0, 0) };
        int maxCostPerWave = 4;
        int maxWavesToSpawn = 4;
        int secondsBetweenWaves = 120;
        //bombard
        string bombardLogicBlock;

        //detection settings
        double activeAntennaDetectionRange = 0;
        double activeReactorDetectionRange = 0;
        double visualDetectionRange = 0;
        double playerDetectionRange = 0;
        double cheatDetectionRange = 0;
        #endregion
        
        /**** INTERNAL - DO NOT MODIFY BELOW HERE ****/
        #region TypeDefs
        public enum Tactic
        {
            Default, Small, Large, Carrier, Boss, Bombard, GravityBombard, Ram, None
        };

        struct TargetData
        {
            public Vector3D position;
            public int threatLevel;
            public long gridId;
            public long blockId;
            public long lastActiveTick;
            public MyDetectedEntityInfo mdei;
        }

        struct AllyData
        {
            public long addr;
            public long gridId;
            public long lastUpdatedTick;
            public int rank;
            public double range;
            public Vector3D location;
            public MyDetectedEntityInfo mdei;
            public Tactic tactic;
            public List<Command> cmds;
            public int currentCommandIndex;
        }

        struct Command
        {
            public int priority;
            public CommandType type;
            public long targetId;
            public Vector3D targetVec;
            public Status status;

            public Command(int priority, CommandType ct, long targetId, Vector3D targetVec, Status status = Status.New)
            {
                this.priority = priority;
                this.type = ct;
                this.targetId = targetId;
                this.targetVec = targetVec;
                this.status = status;
            }

            public Command(int priority, CommandType type) : this(priority, type, 0, Vector3D.Zero) { }
        }

        struct CommandTrigger
        {
            public int index;
            public long id;
            public CommandTriggerCause reason;

            public CommandTrigger(int index, long id, CommandTriggerCause reason)
            {
                this.index = index;
                this.id = id;
                this.reason = reason;
            }
        }
        
        public enum CommandTriggerCause
        {
            NewEnemy, RemovedEnemy, NewAlly, CommandUpdate
        };

        public enum CommandType
        {
            None, Info, Move, MoveVia, MovePrecise, Formation, FormationLead, Attack, Bombard, Scan, Despawn
        };
        CommandType[] transitiveCommandTypes = { CommandType.Scan, CommandType.FormationLead, CommandType.MoveVia };

        public enum Status
        {
            New, Started, Finished, Impossible
        };

        struct PrefabData
        {
            public string id;
            public int cost;

            public PrefabData(string id, int cost)
            {
                this.id = id;
                this.cost = cost;
            }
        }
        #endregion

        #region Fields
        int updateFrequency;
        int executionFrequency;
        long tickTimer;
        long lastRunTick;
        long lastAllyScan;
        long commanderGridId;

        List<IMyRemoteControl> remoteControls = null;
        List<IMyRadioAntenna> antennae = null;
        List<IMyCameraBlock> cameras = null;
        IMyRemoteControl activeRC = null;

        bool isCommander;
        double MOVE_WAYPOINT_TOLERANCE = 50;
        double FORMATION_WAYPOINT_TOLERANCE = 20;
        Queue<CommandTrigger> commandTriggers;
        double DISTANCE_FROM_PLAYER_TO_DESPAWN = 5000;

        double maxDetectionRange;
        HashSet<long> knownNPCGridIds = new HashSet<long>();
        HashSet<long> knownPlayerGridIds = new HashSet<long>();
        const double NPC_RANGE_ADD = 1000;
        long SECONDS_BEFORE_MARK_INACTIVE = 120;

        const double ALLY_RANGE_MULT = 1000;
        const long ALLY_REFRESH_FREQUENCY = 1000;
        IMyBroadcastListener handshakeListener;

        const string COMM_HANDSHAKE = "HANDSHAKE";
        const string COMM_STATUS = "STATUS";
        const string COMM_COMMAND = "CMD";

        const string FORMATION_PB_NAME = "FormationPB";
        IMyProgrammableBlock formationPB;
        int formationStatus = 0;

        Random rand;

        long bombardStartTime;
        long lastSpawnTime;
        int waveSpawnsRemaining;
        IMyProgrammableBlock bombardPB = null;
        IMyTimerBlock bombardTimer = null;
        IMyBlockGroup bombardGroup = null;

        Dictionary<long, TargetData> knownTargets = new Dictionary<long, TargetData>();
        Dictionary<long, AllyData> knownAllies = new Dictionary<long, AllyData>();
        Dictionary<long, long> commIdLookup = new Dictionary<long, long>();
        #endregion

        #region Core
        public Program()
        {
            tickTimer = 0;
            lastRunTick = 0;
            bombardStartTime = 0;
            lastSpawnTime = 0;
            lastAllyScan = 0;
            handshakeListener = IGC.RegisterBroadcastListener(COMM_HANDSHAKE);
            handshakeListener.SetMessageCallback(COMM_HANDSHAKE);
            isCommander = true;
            myData.gridId = Me.CubeGrid.EntityId;
            myData.cmds = new List<Command>();
            myData.currentCommandIndex = -1;
            commandTriggers = new Queue<CommandTrigger>();
            setExecutionFrequency(50);
            maxDetectionRange = Math.Max(activeAntennaDetectionRange, max(activeReactorDetectionRange, visualDetectionRange));
            remoteControls = new List<IMyRemoteControl>();
            GridTerminalSystem.GetBlocksOfType<IMyRemoteControl>(remoteControls);
            cameras = new List<IMyCameraBlock>();
            GridTerminalSystem.GetBlocksOfType<IMyCameraBlock>(cameras);
            antennae = new List<IMyRadioAntenna>();
            GridTerminalSystem.GetBlocksOfType<IMyRadioAntenna>(antennae);
            formationPB = (IMyProgrammableBlock)GridTerminalSystem.GetBlockGroupWithName(FORMATION_PB_NAME);
            
            bombardGroup = GridTerminalSystem.GetBlockGroupWithName(bombardLogicBlock);
            if(bombardGroup == null)
            {
                IMyTerminalBlock bombLogic = GridTerminalSystem.GetBlockWithName(bombardLogicBlock);
                if (bombLogic != null)
                {
                    if (bombLogic is IMyProgrammableBlock)
                    {
                        bombardPB = bombLogic as IMyProgrammableBlock;
                    }
                    else if (bombLogic is IMyTimerBlock)
                    {
                        bombardTimer = bombLogic as IMyTimerBlock;
                    }
                }
            }
            waveSpawnsRemaining = maxWavesToSpawn;
            rand = new Random();
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
        }

        void Main(string argument, UpdateType updateSource)
        {
            //if (argument != null && updateSource == UpdateType.IGC)
            //{
            //}
            if (updateSource == UpdateType.Update1 || updateSource == UpdateType.Update10 || updateSource == UpdateType.Update100)
            {
                if (tickTimer >= lastRunTick + executionFrequency)
                {
                    lastRunTick = tickTimer;
                    run();
                }
                tickTimer += updateFrequency;
            }
            else
            {
                handleMessages();
            }
        }

        void run()
        {
            if (!setup())
            {//setup failed - probably couldn't find RC
                return;
            }
            foreach (AllyData ad in knownAllies.Values)
            {
                Echo("ally info| isCommander:" + (commanderGridId == ad.gridId) + " id:" + ad.gridId + " rank:" + ad.rank + " tactic:" + ad.tactic.ToString() + " range:" + ad.range + "\n");
            }
            handleMessages();
            if (isCommander)
            {
                updateEnemies();
                if (knownTargets.Any())
                {
                    updateAllies();
                }
                updateCommands();
            }
            executeCommands();
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
            myData.location = activeRC.GetPosition();
            return true;
        }

        bool elapsedTicksCheck(long a, long wait)
        {
            if(a  + wait >= tickTimer)
            {
                return true;
            }
            return false;
        }

        bool elapsedSecondsCheck(long a, int seconds)
        {
            TimeSpan ts = new TimeSpan(tickTimer - a);
            if(ts.Seconds >= seconds)
            {
                return true;
            }
            return false;
        }
        #endregion

        #region Comms
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
                    TargetData td;
                    if (knownTargets.ContainsKey(cmd.targetId))
                    {
                        td = knownTargets[cmd.targetId];
                    }
                    else
                    {
                        td = new TargetData();
                        td.gridId = cmd.targetId;
                        commandTriggers.Enqueue(new CommandTrigger(0, td.gridId, CommandTriggerCause.NewEnemy));
                    }
                    td.position = cmd.targetVec;
                    td.lastActiveTick = tickTimer;
                    knownTargets[cmd.targetId] = td;
                }
                else
                {// probably command completed notification
                    AllyData? nad = getAllyByCommId(msg.Source);
                    if(nad.HasValue)
                    {
                        AllyData ad = nad.Value;
                        ad.cmds[cmd.priority] = cmd;
                        if(cmd.priority > ad.currentCommandIndex)
                        {
                            ad.currentCommandIndex = cmd.priority;
                        }
                        updateTacticalCapability(ref ad, cmd);
                        knownAllies[ad.gridId] = ad;
                        commandTriggers.Enqueue(new CommandTrigger(cmd.priority, ad.gridId, CommandTriggerCause.CommandUpdate));
                    }
                }
            }
            else if(msg.Source == knownAllies[commanderGridId].addr)
            {//command directive from commander
                setCommand(myData, cmd);
            }
        }

        AllyData? getAllyByCommId(long id)
        {
            if(IGC.Me == id)
            {
                return myData;
            }
            else if(commIdLookup.ContainsKey(id))
            {
                return knownAllies[commIdLookup[id]];
            }
            return null;
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
                updateAlly(ad);
                if (getRankingGrid(sourceRank, sourceGridId, myData.rank, myData.gridId) == sourceGridId)
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
                else if(sourceRank < 0)
                {// comm source is retreating/disabled
                    if(commanderGridId == sourceGridId)
                    {// source is relinquishing command - select new
                        selectNewCommander();
                    }
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

        void selectNewCommander()
        {
            long maxId = myData.gridId;
            int maxRank = myData.rank;
            foreach(AllyData ad in knownAllies.Values)
            {
                long id = getRankingGrid(maxRank, maxId, ad.rank, ad.gridId);
                if(id == ad.gridId)
                {
                    maxId = ad.gridId;
                    maxRank = ad.rank;
                }
            }
            if(maxId == myData.gridId)
            {// I am the new commander
                isCommander = true;
                commanderGridId = myData.gridId;
                IGC.SendBroadcastMessage(COMM_HANDSHAKE, getHandshakeMsgData());
            }
            else
            {// new commander is a known ally
                isCommander = false;
                commanderGridId = maxId;
            }
        }

        MyTuple<long, long, int, int, double> getHandshakeMsgData(MyIGCMessage msg)
        {
            return (MyTuple<long, long, int, int, double>)msg.Data;
        }

        MyTuple<long, long, int, int, double> getHandshakeMsgData()
        {
            return new MyTuple<long, long, int, int, double>(IGC.Me, myData.gridId, myData.rank, (int)myData.tactic, myData.range);
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

        MyTuple<int, int, long, Vector3D> serialiseCommand(Command cmd)
        {
            return new MyTuple<int, int, long, Vector3D>(cmd.priority, (int)cmd.type, cmd.targetId, cmd.targetVec);
        }

        Command deserialiseCommand(MyIGCMessage msg)
        {
            MyTuple<int, int, long, Vector3D> tuple = (MyTuple<int, int, long, Vector3D>)msg.Data;
            return new Command(tuple.Item1, (CommandType)tuple.Item2, tuple.Item3, tuple.Item4);
        }

        MyTuple<int, int, long, Vector3D> targetDataToCommand(TargetData td)
        {
            return new MyTuple<int, int, long, Vector3D>(0, (int)CommandType.Info, td.gridId, td.position);
        }
        #endregion

        #region Entities
        void updateAllies()
        {
            if (tickTimer - lastAllyScan >= ALLY_REFRESH_FREQUENCY)
            {
                bool newAllyFound = false;
                lastAllyScan = tickTimer;
                List<long> alliedGrids = GetAllAlliedGrids(myData.rank * ALLY_RANGE_MULT);
                foreach (long allyGridId in alliedGrids)
                {
                    if (!knownAllies.ContainsKey(allyGridId))
                    {
                        AllyData ad = new AllyData();
                        ad.gridId = allyGridId;
                        ad.rank = -1;
                        updateAlly(ad);
                        newAllyFound = true;
                        commandTriggers.Enqueue(new CommandTrigger(0, allyGridId, CommandTriggerCause.NewAlly));
                    }
                }
                if (newAllyFound)
                {
                    IGC.SendBroadcastMessage(COMM_HANDSHAKE, getHandshakeMsgData());
                }
            }
        }

        void updateEnemies()
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
                    double distanceToTarget = Vector3D.Distance(targetPosition, myData.location);
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
                    {//TODO: cast ray at random location withing target's bounding box
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
                            td.lastActiveTick = tickTimer;
                            commandTriggers.Enqueue(new CommandTrigger(0, td.gridId, CommandTriggerCause.NewEnemy));
                        }
                        if (mdei.IsEmpty())
                        {
                            mdei = GetMDEI(gridId);
                        }
                        td.position = targetPosition;
                        td.blockId = targetBlockId;
                        td.mdei = mdei;
                        td.threatLevel = threatLevel;
                        knownTargets[gridId] = td;
                        if (!isCommander)
                        {//update commander with latest target data
                            IGC.SendUnicastMessage(knownAllies[commanderGridId].addr, COMM_COMMAND, targetDataToCommand(td));
                        }
                    }
                }
            }
            Echo("No valid targets found.");
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

        void updateKnownTargets()
        {//remove old/inactive/disabled targets
            foreach(TargetData td in knownTargets.Values)
            {
                TimeSpan ts = new TimeSpan(tickTimer - td.lastActiveTick);
                if (ts.Seconds > SECONDS_BEFORE_MARK_INACTIVE)
                {//target hasn't been detected as active in too long - remove it
                    knownTargets.Remove(td.gridId);
                    commandTriggers.Enqueue(new CommandTrigger(0, td.gridId, CommandTriggerCause.RemovedEnemy));
                }
            }
        }

        void updateAllyLocation(AllyData ad)
        {
            if(ad.lastUpdatedTick != tickTimer)
            {
                ad.mdei = GetMDEI(ad.gridId);
                ad.location = ad.mdei.Position;
                ad.lastUpdatedTick = tickTimer;
                updateAlly(ad);
            }
        }
        #endregion

        #region Tactical
        void executeCommands()
        {//TODO
            Vector3D grav = activeRC.GetNaturalGravity();
            bool inGravity = false;
            if (grav.Length() > 0)
            {
                inGravity = true;
            }

            bool cont = true;
            CommandType ct;
            Status status;
            Command myCommand;
            Tactic tactic = myData.tactic;
            for (int i = myData.currentCommandIndex; cont && i < myData.cmds.Count; ++i)
            {
                myCommand = myData.cmds[i];
                ct = myCommand.type;
                status = myCommand.status;
                cont = transitiveCommandTypes.Contains(ct);
                if (formationStatus != 0 && ct != CommandType.Formation)
                {// attempt to turn off formation PB
                    if (formationPB != null && formationPB.IsWorking)
                    {
                        if (formationPB.TryRun("stop"))
                        {
                            formationStatus = 0;
                        }
                    }
                    else
                    {
                        formationStatus = 0;
                    }
                }
                if (status == Status.Finished || status == Status.Impossible)
                {
                    continue;
                }

                if (ct == CommandType.None)
                {// Do nothing

                }
                else if (ct == CommandType.Scan)
                {
                    updateEnemies();
                }
                else if (ct == CommandType.Move || ct == CommandType.MoveVia)
                {// Move to target position
                    if (status == Status.New)
                    {//initiate movement
                        flyToLocation(myCommand.targetVec);
                        status = Status.Started;
                    }
                    else if (status == Status.Started)
                    {// Check if end condition met
                        if (distanceLessThan(myData.location, myCommand.targetVec, MOVE_WAYPOINT_TOLERANCE))
                        {//End condition met - inform commander
                            changeMyCommandStatus(myCommand);
                        }
                    }
                }
                else if (ct == CommandType.FormationLead)
                {//I am the leader
                    IGC.SendBroadcastMessage<MyTuple<MatrixD, Vector3D>>("FSLeaderSystem1",
                        new MyTuple<MatrixD, Vector3D>(activeRC.WorldMatrix, activeRC.GetShipVelocities().LinearVelocity));
                }
                else if (ct == CommandType.Formation)
                {// Move into formation with target grid with offset dictated by vec
                    if (status == Status.New)
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
                                    status = Status.Started;
                                }
                            }
                        }
                    }
                }
                else if (ct == CommandType.Bombard)
                {// attack the target from a distance with tactic x
                    long targetId = myCommand.targetId;
                    //probably notify complete command if ammunition/drones exhausted
                    if (tactic == Tactic.Carrier)
                    {//spawn drones etc
                        if(status == Status.New)
                        {
                            lastSpawnTime = tickTimer;
                            myCommand.status = Status.Started;
                            setCommand(myData, myCommand);
                        }
                        else if (elapsedSecondsCheck(lastSpawnTime, secondsBetweenWaves))
                        {
                            if (waveSpawnsRemaining > 0)
                            {
                                spawnPrefabWave();
                            }
                            else
                            {
                                changeMyCommandStatus(myCommand, Status.Impossible);
                            }
                        }
                    }
                    else if (tactic == Tactic.Bombard)
                    {// move to position at range then activate bombard logic

                    }
                    else if (tactic == Tactic.GravityBombard)
                    {// move to position above target and activate bombard logic

                    }
                }
                else if (ct == CommandType.Attack)
                {// engage target with tactic x

                }
                else if (ct == CommandType.Despawn)
                {
                    despawnIfPossible();
                }
            }

        }

        void despawnIfPossible()
        {
            Vector3D nearestPlayer = new Vector3D(0, 0, 0);
            bool playerExists = activeRC.GetNearestPlayer(out nearestPlayer);
            if (!playerExists || Vector3D.Distance(myData.location, nearestPlayer) > DISTANCE_FROM_PLAYER_TO_DESPAWN)
            {
                //SendChatMessage("Attempting Despawn", "Space Pirates");
                AttemptDespawn();
            }
        }

        void enterFormation(long targetId, Vector3D offset)
        {//unused backup
            Vector3D leadPos = GetTrackedEntityPosition(targetId);
            leadPos += offset;
            flyToLocation(leadPos);
        }

        void spawnPrefabWave()
        {
            int waveCost = 0;
            List<PrefabData> prefabs = new List<PrefabData>();
            int index = rand.Next(spawns.Length);
            while(waveCost < maxCostPerWave && prefabs.Count < prefabOffsets.Length)
            {
                PrefabData pd = spawns[index];
                prefabs.Add(pd);
                waveCost += pd.cost;
            }
                
            for(int i = 0; i < prefabs.Count; ++i)
            {
                PrefabData pd = prefabs[i];
                SpawnReinforcements("Prefab", pd.id, Me.GetOwnerFactionTag(), true, getWorldPositionRelativeToMe(prefabOffsets[i]), 
                    activeRC.WorldMatrix.Forward, activeRC.WorldMatrix.Up, activeRC.GetShipVelocities().LinearVelocity);
            }
            --waveSpawnsRemaining;            
        }
        #endregion

        #region Command
        void updateCommands()
        {
            if (knownTargets.Any())
            {
                issueCommands();
            }
            else
            {//form up on commander if no targets
                foreach (AllyData ad in knownAllies.Values)
                {
                    Command cmd = ad.cmds[0];
                    if (cmd.type != CommandType.Formation)
                    {
                        cmd = getFormationCommand(ad, myData.gridId);
                        setCommand(ad, cmd);
                    }
                }
            }
        }

        void issueCommands()
        {
            CommandTrigger trigger;
            while (commandTriggers.Any())
            {
                trigger = commandTriggers.Dequeue();
                CommandTriggerCause reason = trigger.reason;
                if (reason == CommandTriggerCause.NewAlly)
                {
                    issueCommandsForNewAlly(knownAllies[trigger.id]);
                }
                else if(reason == CommandTriggerCause.NewEnemy)
                {// new target found - probably do something
                    if(knownTargets.ContainsKey(trigger.id))
                    {
                        issueCommandsForNewTarget(knownTargets[trigger.id]);
                    }
                    
                }
                else if(reason == CommandTriggerCause.RemovedEnemy)
                {// target no longer applicable - adjust any commands targeting it
                    issueCommandsForRemovedTarget(trigger.id);
                }
                else if(reason == CommandTriggerCause.CommandUpdate)
                {// command completed or impossible
                    if(knownAllies.ContainsKey(trigger.id))
                    {
                        AllyData ad = knownAllies[trigger.id];
                        if(ad.cmds.Count > trigger.index)
                        {
                            Command cmd = ad.cmds[trigger.index];
                            issueCommandsForUpdatedCommand(ad, cmd);
                        }
                    }
                }
            }
        }

        void issueCommandsForNewAlly(AllyData ad)
        {
            if(knownTargets.Any())
            {
                assignNewTarget(ad);
            }
        }

        void issueCommandsForNewTarget(TargetData td)
        {
            bool hasTarget;
            foreach(AllyData ad in knownAllies.Values)
            {
                hasTarget = false;
                for(int i = ad.currentCommandIndex; i < ad.cmds.Count; ++i)
                {
                    Command cmd = ad.cmds[i];
                    if(cmd.type == CommandType.Attack || cmd.type == CommandType.Bombard)
                    {
                        hasTarget = true;
                        break;
                    }
                }
                if(!hasTarget)
                {
                    assignNewTarget(ad, td);
                }
            }
        }

        void issueCommandsForRemovedTarget(long id)
        {
            TargetData td;
            bool alternateExists = findTarget(out td, id);
            if (alternateExists)
            {
                foreach (AllyData ad in knownAllies.Values)
                {
                    for (int i = ad.currentCommandIndex; i < ad.cmds.Count; ++i)
                    {
                        Command cmd = ad.cmds[i];
                        if ((cmd.type == CommandType.Attack || cmd.type == CommandType.Bombard))
                        {
                            if (id == cmd.targetId)
                            {
                                assignNewTarget(ad, td);
                            }
                            break;
                        }
                    }
                }
            }
        }

        void issueCommandsForUpdatedCommand(AllyData ad, Command cmd)
        {TODO
            if(cmd.status == Status.Finished)
            {
                if(cmd.type == CommandType.Attack)
                {//need a target
                    TargetData td;
                    bool hasAlternate = findTarget(out td, cmd.targetId);
                    if (hasAlternate)
                    {
                        assignNewTarget(ad, td);
                    }
                    else
                    {// no more viable targets
                        updateAllyLocation(ad);
                        retreat(ad);
                    }
                }
                else if(cmd.type == CommandType.Move || cmd.type == CommandType.MovePrecise)
                {//maybe enter formation
                    
                }

            }
            else if(cmd.status == Status.Impossible)
            {
                
                if(cmd.type == CommandType.Attack)
                {// probably disabled or out of ammo
                    ram(ad);
                }
                else if (cmd.type == CommandType.Bombard)
                {// bombard resources depleted - attack or retreat
                    postBombard(ad, cmd.targetId);
                }
            }
        }

        void postBombard(AllyData ad, long targetId)
        {
            if (rand.Next(2) == 0)
            {
                updateAllyLocation(ad);
                retreat(ad, targetId);
            }
            else
            {
                assignNewTarget(ad, knownTargets[targetId]);
            }
        }

        void retreat(AllyData ad, long gridId)
        {//retreat from an enemy grid with id gridId
            retreat(ad, knownTargets[gridId].position);
        }

        void retreat(AllyData ad)
        {//retreat backwards
            retreat(ad, ad.mdei.Orientation.Forward);
        }

        void retreat(AllyData ad, Vector3D avoid)
        {
            Command retreatMoveCommand = new Command(0, CommandType.MovePrecise);
            retreatMoveCommand.targetVec = getRetreatVector(ad.location, avoid);
            setCommand(ad, retreatMoveCommand);
            Command retreatCommand = new Command(1, CommandType.Despawn);
            setCommand(ad, retreatCommand);
        }

        bool findTarget(out TargetData ret, long targetToAvoid = 0)
        {
            ret = new TargetData();
            bool targetFound = false;
            foreach(TargetData td in knownTargets.Values)
            {
                if(td.gridId != targetToAvoid)
                {
                    ret = td;
                    targetFound = true;
                    break;
                }
            }
            return targetFound;
        }

        void ram(AllyData ad)
        {TODO

        }

        void assignNewTarget(AllyData ad)
        {
            TargetData td;
            if(findTarget(out td))
            {
                assignNewTarget(ad, td);
            }
        }

        void assignNewTarget(AllyData ad, TargetData td)
        {
            if(ad.tactic == Tactic.Bombard)
            {
                if (distanceLessThan(ad.location, td.position, ad.range))
                {
                    //start bombarding new target
                    setCommand(ad, new Command(0, CommandType.Bombard, td.gridId, td.position));
                }
                else
                {
                    setCommand(ad, new Command(0, CommandType.Move, 0, calculateBombardPosition(ad)));
                    setCommand(ad, new Command(1, CommandType.Bombard, td.gridId, td.position));
                    //move to bombardment range
                }
            }
        }

        void setCommand(AllyData ad, Command cmd)
        {
            if (cmd.type != CommandType.Info)
            {
                List<Command> cmds = ad.cmds;
                cmds[cmd.priority] = cmd;
                int removeStartIndex = cmd.priority + 1;
                if (removeStartIndex < cmds.Count + 1)
                {
                    cmds.RemoveRange(removeStartIndex, cmds.Count - removeStartIndex);
                }
                if (cmd.priority <= ad.currentCommandIndex)
                {
                    ad.currentCommandIndex = cmd.priority;
                    if (ad.Equals(myData))
                    {//updating me
                        setRunSpeed(cmd.type);
                    }
                }
                if(!ad.Equals(myData))
                {//not me
                    sendCommand(ad.addr, cmd);
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
            else if (ct == CommandType.Formation)
            {
                newExecutionFrequency = 5;
            }
            else if (ct == CommandType.Attack)
            {
                newExecutionFrequency = 1;
            }
            else if (ct == CommandType.Bombard)
            {
                newExecutionFrequency = 100;
            }

            if (executionFrequency != newExecutionFrequency)
            {
                setExecutionFrequency(executionFrequency);
            }
        }

        Command getFormationCommand(AllyData ad, long target)
        {
            return new Command(0, CommandType.Formation, target, new Vector3D(ad.rank * 100, 0, 0));
        }

        void changeMyCommandStatus(Command cmd, Status status = Status.Finished)
        {
            cmd.status = status;
            myData.cmds[cmd.priority] = cmd;
            updateTacticalCapability(ref myData, cmd);
            sendCommand(knownAllies[commanderGridId].addr, cmd);
        }

        bool updateTacticalCapability(ref AllyData ad, Command cmd)
        {
            if (cmd.status == Status.Impossible)
            {//ally can probably no longer perform some tactics
                Tactic updated = ad.tactic;
                if (cmd.type == CommandType.Bombard)
                {
                    updated = Tactic.Default;
                }
                else if (cmd.type == CommandType.Attack)
                {
                    if (ad.tactic == Tactic.Default)
                    {
                        updated = Tactic.Ram;
                    }
                    else if(ad.tactic == Tactic.Ram)
                    {
                        updated = Tactic.None;
                    }
                }
                if (updated != ad.tactic)
                {
                    ad.tactic = updated;
                    return true;
                }
            }
            return false;
        }

        #endregion

        #region Nav
        bool inGravity()
        {
            return activeRC.GetNaturalGravity().Length() > 0;
        }
        Vector3D calculateBombardPosition(AllyData ad, TargetData td)
        {
            Vector3D vec;
            if(inGravity())
            {
                //get bombard location myData.range meters away, 33 degrees above horizon
            }
            else
            {
                vec =  interpolateByDistance(ad.location, td.position, myData.range);
            }
            return vec;
        }

        Vector3D calculateGravityBombardPosition(AllyData ad, TargetData td)
        {
            Vector3D planet;
            bool nearPlanet = activeRC.TryGetPlanetPosition(out planet);
            return interpolateByDistance();
        }

        Vector3D getRetreatVector(Vector3D start, Vector3D avoid)
        {
            return interpolateByDistance(avoid, start, DISTANCE_FROM_PLAYER_TO_DESPAWN * 2);
        }

        Vector3D interpolateByDistance(Vector3D a, Vector3D b, double distance)
        {
            Vector3D aNormalizedToB = Vector3D.Normalize(a - b);
            return a + aNormalizedToB * distance;
        }

        Vector3D getWorldPositionRelativeToMe(Vector3D offset)
        {
            return Vector3D.Transform(offset, activeRC.WorldMatrix);
        }

        void addGPSToRC(Command cmd)
        {
            activeRC.AddWaypoint(cmd.targetVec, cmd.priority.ToString());
        }

        void goToLocation(Vector3D gps, bool precisionMode)
        {
            if(activeRC.HasWheels)
            {
                driveToLocation(gps);
            }
            else
            {
                flyToLocation(gps, precisionMode);
            }
        }

        void driveToLocation(Vector3D gps)
        {//TODO

        }

        void flyToLocation(Vector3D coords, bool precisionMode = true, float speedLimit = 20, bool collisionAvoidance = true)
        {
            activeRC.ClearWaypoints();
            activeRC.AddWaypoint(coords, "1");
            activeRC.FlightMode = FlightMode.OneWay;
            activeRC.IsMainCockpit = true;
            activeRC.SetAutoPilotEnabled(true);
            activeRC.SetCollisionAvoidance(collisionAvoidance);
            activeRC.SetDockingMode(precisionMode);
            activeRC.SpeedLimit = speedLimit;
        }
        #endregion

        #region Support
        
        long getRankingGrid(int rankA, long idA, int rankB, long idB)
        {
            if(rankA > rankB || (rankA == rankB && idA < idB))
            {
                return idA;
            }
            return idB;
        }

        void updateAlly(AllyData ad)
        {
            if (ad.addr != 0)
            {
                if(!commIdLookup.ContainsKey(ad.addr))
                {
                    commIdLookup[ad.addr] = ad.gridId;
                }
            }
            knownAllies[ad.gridId] = ad;
        }

        void removeAlly(AllyData ad)
        {
            if(knownAllies.ContainsKey(ad.gridId))
            {
                knownAllies.Remove(ad.gridId);
            }
            if (commIdLookup.ContainsKey(ad.addr))
            {
                commIdLookup.Remove(ad.addr);
            }
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
        #endregion

        #region MES
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
        #endregion

        #region footer
        /////////////////////////////////
    }
}
#endregion