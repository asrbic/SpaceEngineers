void Main() {
	IMyCameraBlock camera = getCamera();
	if(camera != null) {
		Echo("Recon camera found");
		Echo("Range: " + camera.AvailableScanRange);
		Echo("canscan:" + camera.CanScan(10));
	}
}

IMyCameraBlock getCamera() {
	//TODO error checking
	return GridTerminalSystem.GetBlockWithName("Camera") as IMyCameraBlock;
}