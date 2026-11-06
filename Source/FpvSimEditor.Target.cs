using UnrealBuildTool;

public class FpvSimEditorTarget : TargetRules
{
	public FpvSimEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("FpvSim");
	}
}
