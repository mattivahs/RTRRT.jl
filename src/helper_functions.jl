using LinearAlgebra
using Plots

struct parameters
    gamma
    x_range
    y_range
    max_tree_nodes
    nominal_step
    expansion_time
    RewireTime
end

function calc_edge_cost(p1::Vector{Float64}, p2::Vector{Float64})
    "Euclidean distance edge cost"
    return norm(p2[1:2] - p1[1:2])
end

function sample_point(parameters)
    "Sample uniformly in workspace"
    x = parameters.x_range[1] + (parameters.x_range[2] - parameters.x_range[1]) * rand()
    y = parameters.y_range[1] + (parameters.y_range[2] - parameters.y_range[1]) * rand()
    return [x, y]
end

function plot_tree(tree)
    x_data = []
    y_data = []

    for node in tree
        if node.parent_id != 0
            push!(x_data, [tree[node.parent_id].pose[1], node.pose[1]])
            push!(y_data, [tree[node.parent_id].pose[2], node.pose[2]])
        end
    end
    plot(x_data, y_data, marker=(:circle,2), legend=false, color="grey")
    plot!([tree[1].pose[1]], [tree[1].pose[2]], marker=(:circle,5), legend=false, color="red")
end
