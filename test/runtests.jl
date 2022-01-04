using RTRRT
using Test

init_pose = [0.5, 2.0, 0]
params = RTRRT.parameters(1.0, [0.0, 10.0], [0.0, 10.0], 5000, 0.4, 0.1, 1.0, 20)

tree = RTRRT.Tree(init_pose, params)
for i = 1:1
    RTRRT.expansion(tree)
    # RTRRT.rewire(tree)
    RTRRT.rewire_from_root(tree)
end
RTRRT.plot_tree(tree.all_nodes)
RTRRT.visualize_map(tree.map)
@testset "RTRRT.jl" begin
    # Write your tests here
end
