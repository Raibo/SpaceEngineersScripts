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
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ObjectBuilders.Definitions;
using VRage.Game;
using VRage;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        static IMyTextSurface Lcd;
        IMyRemoteControl Cock;
        private IMyTerminalBlock Tip;

        private RoboticArm roboticArm;

        public Program()
        {
            var allBlocks = new List<IMyTerminalBlock>();
            GridTerminalSystem.GetBlocks(allBlocks);
            
            Lcd = GetBlock<IMyTextSurface>(allBlocks, x => x.CustomName == "lcd controls");
            Cock = GetBlock<IMyRemoteControl>(allBlocks, x => x.CustomName.Contains("[ra c]"));
            Tip = GetBlock<IMyTerminalBlock>(allBlocks, x => x.CustomName.Contains("[ra t]"));

            var rotationRotor = GetBlock<IMyMotorStator>(allBlocks, x => x.CustomName.Contains("[ra r]"));
            roboticArm = new RoboticArm(allBlocks, rotationRotor, Tip);
            roboticArm.lcd = Lcd;

            Lcd.WriteText("Hello world!");

            Runtime.UpdateFrequency = UpdateFrequency.Update10;
        }

        T GetBlock<T>(List<IMyTerminalBlock> allBlocks, Func<IMyTerminalBlock, bool> predicate) where T: class =>
            allBlocks.FirstOrDefault(x => x is T && predicate(x)) as T;

        public void Save()
        {

        }

        public void Main(string argument, UpdateType updateSource)
        {
            roboticArm.KeepMoving(new Vector3D(53539.59, -26784.67, 11963.55), 1);
        }
    }
}
