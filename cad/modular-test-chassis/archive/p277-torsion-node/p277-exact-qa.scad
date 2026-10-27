// Exact-solid P2-77 checks. Empty STL output is PASS.
use <../../p277-torsion-node.scad>
use <../../shared-flywheel-p273-bolted-ski.scad>

qa="ski_brackets";

if (qa=="ski_brackets") intersection() {
    repair_structure();
    union() for(side=[-1,1]) p273_mount_bracket(side);
}
