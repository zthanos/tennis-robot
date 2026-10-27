// P2-85 integration gate: the taller ski brackets and their moving nylocs
// must remain clear of both the released P2-84 rear panels and the complete
// body concept F front/side/rear skin.

use <shared-flywheel-p273-bolted-ski.scad>
use <../body-panels/body-panels-f-rear-v2.scad>
use <../body-panels/body-panels-f.scad>

p285_panel_qa = "rear";
front_on = true;  // force the F front continuation into the concept gate

module p285_bracket_and_nyloc_envelope() {
    p285_brackets();
    for (h=[0:1:20]) p273_fastener_envelopes(h);
}

if (p285_panel_qa=="rear") intersection() {
    p285_bracket_and_nyloc_envelope();
    all_f_rear(0);
}
else if (p285_panel_qa=="concept_f") intersection() {
    p285_bracket_and_nyloc_envelope();
    tub_solid();
}
else assert(false,str("unknown p285_panel_qa ",p285_panel_qa));
