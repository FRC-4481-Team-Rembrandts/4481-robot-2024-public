package frc.team4481.robot.util;

import java.util.ArrayList;
import java.util.List;

public class LookupTableConfig {
    private List<LookupTablePreset> lookupTableConfigs;

    public LookupTableConfig(){
        lookupTableConfigs = new ArrayList<>();
    }

    public LookupTableConfig(ArrayList<LookupTablePreset> plookupTableConfigs){
        this.lookupTableConfigs = plookupTableConfigs;
    }

    public List<LookupTablePreset> getLookupTableConfigs() {
        return lookupTableConfigs;
    }
}
