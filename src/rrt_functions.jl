using DataStructures
using NearestNeighbors

include("map.jl")


mutable struct Tree
    init_pose::Vector{Float64}
    params::parameters
    current_goal::Vector{Float64}
    all_nodes
    QrPrior::Stack{Int}
    Qr::Queue{Int}
    Qs::Queue{Int}
    # Passcode to prevent unneccessary looping
    passcode::Float64
    positions
    kdtree
    map::PlanningMap

    function Tree(init_pose::Vector{Float64}, params::parameters)
        println("Initializing RT-RRT* ...")
        root_node = TreeNode(1, init_pose, 0)
        QrPrior = Stack{Int}()
        Qr = Queue{Int}()
        Qs = Queue{Int}()
        passcode = rand()
        kdtree = KDTree(init_pose[1:2])
        positions = init_pose[1:2]
        map = PlanningMap(params)
        load_default_map(map, params)
        new(init_pose, params, [1, 1], [root_node], QrPrior, Qr, Qs, passcode, positions, kdtree, map)
    end
end


mutable struct TreeNode
    "ID in tree"
    id::Int64
    "Pose of the 2D robot"
    pose::Vector{Float64}
    "Parent Node"
    parent_id::Int64
    "Total cost = Parent Cost + edge_parent"
    totalcost::Float64
    "Edge cost"
    edge_cost::Float64
    "Child IDs"
    child_ids::Vector{Int64}
    "Passcode (has already been processed in rewire from root?)"
    passcode::Float64

    function TreeNode(id::Int64, pose::Vector{Float64}, parent_id::Int)
        new(id, pose, parent_id, 0, 0, [], 0)
    end
end

function steer(closest::TreeNode, pos::Vector{Float64}, params::parameters)
    delta = [pos[1] - closest.pose[1], pos[2] - closest.pose[2]]
    delta /= norm(delta)
    new_pose = [clamp(closest.pose[1] + delta[1] * params.nominal_step, params.x_range[1], params.x_range[2]),
                clamp(closest.pose[2] + delta[2] * params.nominal_step, params.y_range[1], params.y_range[2]),
                closest.pose[3] + atan(delta[2]/delta[1])]
    return new_pose
end

function near_nodes(this::Tree, node)
    "Naive Implementation, KD Trees will follow"
    r = 0.8
    nearnode_ids = inrange(this.kdtree, node.pose[1:2], r)
    return nearnode_ids
end

function expansion(this::Tree)
    i = 0
    start_time = time()
    while size(this.all_nodes, 1) <= this.params.max_tree_nodes
        println("Iteration $i")
        sampled_pos = sample_point(this.params)
        # dist_, closest_ind = findmin(map(x->norm(this.all_nodes[x].pose[1:2] - sampled_pos), 1:size(this.all_nodes, 1)))
        # Nearest node obtained through KD Tree
        closest_ind, dist_ = knn(this.kdtree, sampled_pos, 1)
        closest_ind = closest_ind[1]
        new_pose = steer(this.all_nodes[closest_ind], sampled_pos, this.params)

        if !edge_collision(this.map, this.all_nodes[closest_ind].pose[1:2], new_pose[1:2])
            # Create new node
            new_node = TreeNode(size(this.all_nodes, 1) + 1, new_pose, closest_ind)
            new_node.edge_cost = calc_edge_cost(this.all_nodes[closest_ind].pose, new_pose)
            new_node.totalcost = this.all_nodes[closest_ind].totalcost + new_node.edge_cost
            push!(this.all_nodes[new_node.parent_id].child_ids, new_node.id)

            # nearnodes = near_nodes(this.all_nodes, new_node, this.params.gamma)
            nearnode_ids = near_nodes(this, new_node)
            c_min = new_node.totalcost
            new_parent_id = new_node.parent_id
            parent_changed = false
            for node_id in nearnode_ids
                c_new = this.all_nodes[node_id].totalcost + calc_edge_cost(this.all_nodes[node_id].pose, new_node.pose)
                if c_new < c_min
                    if !edge_collision(this.map, this.all_nodes[node_id].pose[1:2], new_node.pose[1:2])
                        new_parent_id = node_id
                        c_min = c_new
                        parent_changed = true
                    end
                end
            end

            push!(this.all_nodes, new_node)
            update_kdtree(this)

            if parent_changed
                change_parent(this, new_node.id, new_parent_id)
            else
                change_parent(this, new_node.id, new_node.parent_id)
            end

            push!(this.QrPrior, size(this.all_nodes, 1))
            i += 1
        end
    end
end

function rewire(this::Tree)
    start_time = time()
    i = 0
    while !(isempty(this.QrPrior) && isempty(this.Qr)) #&& (time() - start_time) <= this.params.RewireTime
        println("Rewire Iteration $i")
        if !isempty(this.QrPrior)
            rewire_ind = pop!(this.QrPrior)
        else
            rewire_ind = dequeue!(this.Qr)
        end
        # nearnodes = near_nodes(this.all_nodes, this.all_nodes[rewire_ind], this.params.gamma)
        nearnode_ids = near_nodes(this, this.all_nodes[rewire_ind])
        # Boolean value for changing parent
        parent_changed = false

        for node_id in nearnode_ids
            # New costs when changing parent
            c_new = this.all_nodes[rewire_ind].totalcost + calc_edge_cost(this.all_nodes[rewire_ind].pose, this.all_nodes[node_id].pose)

            # update parent
            if c_new < this.all_nodes[node_id].totalcost
                if !edge_collision(this.map, this.all_nodes[node_id].pose[1:2], this.all_nodes[rewire_ind].pose[1:2])
                    change_parent(this, node_id, rewire_ind)
                    parent_changed = true
                    enqueue!(this.Qr, node_id)
                end
            end

            if parent_changed
                propagate_costs(this, node_id)
            end
        end
        i += 1
    end
end

function rewire_from_root(this::Tree)
    start_time = time()

    if isempty(this.Qs)
        enqueue!(this.Qs, 1)
    end
    i = 0
    while !isempty(this.Qs) #&& (time() - start_time) <= this.params.RewireTime
        println("Rewire From Root ITERATION $i")
        rewire_ind = dequeue!(this.Qs)

        # nearnodes = near_nodes(this.all_nodes, this.all_nodes[rewire_ind], this.params.gamma)
        nearnode_ids = near_nodes(this, this.all_nodes[rewire_ind])

        for node_id in nearnode_ids
            # New costs when changing parent
            c_new = this.all_nodes[rewire_ind].totalcost + calc_edge_cost(this.all_nodes[rewire_ind].pose, this.all_nodes[node_id].pose)

            parent_changed = false
            # update parent
            if c_new < this.all_nodes[node_id].totalcost
                if !edge_collision(this.map, this.all_nodes[node_id].pose[1:2], this.all_nodes[rewire_ind].pose[1:2])
                    change_parent(this, node_id, rewire_ind)
                    parent_changed = true
                end
            end
            if this.all_nodes[node_id].passcode != this.passcode
                enqueue!(this.Qs, node_id)
                this.all_nodes[node_id].passcode = this.passcode
            end

            if parent_changed
                # Propagate costs
                propagate_costs(this, node_id)
            end
        end
        i += 1
    end
end

function propagate_costs(this::Tree, id::Int64)
    # Recursion
    if !isempty(this.all_nodes[id].child_ids)
        for child_id in this.all_nodes[id].child_ids
            # Update own costs
            this.all_nodes[child_id].totalcost = this.all_nodes[id].totalcost + this.all_nodes[child_id].edge_cost
            propagate_costs(this, child_id)
            println("PRPAGATING COST")
        end
    end
end

function change_parent(this::Tree, node_id, new_parent_id)
    # Update child id's of parent node
    setdiff!(this.all_nodes[this.all_nodes[node_id].parent_id].child_ids, node_id)
    this.all_nodes[node_id].parent_id = new_parent_id
    this.all_nodes[node_id].edge_cost = calc_edge_cost(this.all_nodes[new_parent_id].pose, this.all_nodes[node_id].pose)
    this.all_nodes[node_id].totalcost = this.all_nodes[new_parent_id].totalcost + this.all_nodes[node_id].edge_cost
    push!(this.all_nodes[new_parent_id].child_ids, node_id)
end

function update_costs(this::Tree, node_id::Int64)
    parent_id = this.all_nodes[node_id].parent_id
    this.all_nodes[node_id].edge_cost = calc_edge_cost(his.all_nodes[parent_id].pose, this.all_nodes[node_id].pose)
    this.all_nodes[nodes_id].totalcost = this.all_nodes[parent_id].totalcost + this.all_nodes[node_id].edge_cost
end

function update_kdtree(this::Tree)
    this.positions = cat(this.positions, this.all_nodes[end].pose[1:2], dims=2)
    this.kdtree = KDTree(this.positions)
end
