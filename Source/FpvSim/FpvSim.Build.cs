using UnrealBuildTool;

public class FpvSim : ModuleRules
{
	public FpvSim(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "ProceduralMeshComponent", "Json", "CesiumRuntime" });
		PublicSystemLibraries.AddRange(new string[] { "dinput8.lib", "dxguid.lib" });   // DirectInput: the radio in USB joystick mode
	}
}
