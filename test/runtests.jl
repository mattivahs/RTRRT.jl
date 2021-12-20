using RTRRT
using Test

init_pose = [0.5, 2.0, 0]
params = RTRRT.parameters(1, [0, 10], [0, 10], 10000, 0.4, 0.1, 1)

tree = RTRRT.Tree(init_pose, params)
for i = 1:1
    RTRRT.expansion(tree)
    # RTRRT.rewire(tree)
    RTRRT.rewire_from_root(tree)
end
RTRRT.plot_tree(tree.all_nodes)
@testset "RTRRT.jl" begin
    # Write your tests here
end
