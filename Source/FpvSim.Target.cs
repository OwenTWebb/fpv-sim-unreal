using UnrealBuildTool;

public class FpvSimTarget : TargetRules
{
	public FpvSimTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("FpvSim");
	}
}
