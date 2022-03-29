using CSFML.LibCSFML
using RTRRT
using LinearAlgebra
using Profile

function CoordsToPixel(p, params)
    fac_x = 1280.0 / (params.x_range[2] - params.x_range[1])
    fac_y = 720.0 / (params.y_range[2] - params.y_range[1])
    x_v = Int(round((p[1] - params.x_range[1]) * fac_x))
    y_v = Int(round((p[2] - params.y_range[1]) * fac_y))
    return sfVector2f(x_v, y_v)
end

function PixelToCoords(p, params)
    fac_x = (params.x_range[2] - params.x_range[1]) / 1280.0
    fac_y = (params.y_range[2] - params.y_range[1]) / 720.0
    x_c = p.x * fac_x + params.x_range[1]
    y_c = p.y * fac_y + params.y_range[1]
    return [x_c, y_c]
end

function update_edges(edges, tree)
    sfVertexArray_clear(edges)
    for node in tree.all_nodes
        if node.parent_id != 0
            pos = CoordsToPixel(node.pose[1:2], tree.params)
            sfVertexArray_append(
                edges,
                sfVertex(pos, sfColor_fromRGB(180, 180, 180), pos),
            )
            pos = CoordsToPixel(
                tree.all_nodes[node.parent_id].pose[1:2],
                tree.params,
            )
            sfVertexArray_append(
                edges,
                sfVertex(pos, sfColor_fromRGB(180, 180, 180), pos),
            )
        end
    end
end

function update_goal_path(goal_path, tree)
    sfVertexArray_clear(goal_path)
    new_goal_path = RTRRT.extract_goal_path(tree)
    goal_path_circles = []
    if length(new_goal_path) > 1
        for i = 1:length(new_goal_path)-1
            pos = CoordsToPixel(new_goal_path[i], tree.params)
            sfVertexArray_append(
                goal_path,
                sfVertex(pos, sfColor_fromRGB(181, 26, 14), pos),
            )
            pos = CoordsToPixel(new_goal_path[i+1], tree.params)
            sfVertexArray_append(
                goal_path,
                sfVertex(pos, sfColor_fromRGB(181, 26, 14), pos),
            )
            push!(goal_path_circles, sfCircleShape_create())
            sfCircleShape_setRadius(goal_path_circles[end], 3)
            sfCircleShape_setPosition(
                goal_path_circles[end],
                pos)
            sfCircleShape_setFillColor(goal_path_circles[end], sfColor_fromRGB(181, 26, 14))

        end
    end
    return goal_path_circles
end

function track_path(tree)
    goal_path = RTRRT.extract_goal_path(tree)
    current_pos = tree.all_nodes[1].pose[1:2]
    if length(goal_path) < 2
        return 0.0, 0.0
    end
    goal_pos = goal_path[end-1]

    delta = 0.1 * (goal_pos[1:2] - current_pos) / norm(goal_pos[1:2] - current_pos)
    return delta[1], delta[2]
end

function process_events(tree)
    dx = 0.0
    dy = 0.0
    delta = 0.1
    has_changed = false
    if Bool(sfKeyboard_isKeyPressed(sfKeySpace))
        dx, dy = track_path(tree)
        has_changed = true
    else
        if Bool(sfKeyboard_isKeyPressed(sfKeyD))
            dx += delta
            has_changed = true
        elseif Bool(sfKeyboard_isKeyPressed(sfKeyA))
            dx -= delta
            has_changed = true
        end
        if Bool(sfKeyboard_isKeyPressed(sfKeyS))
            dy += delta
            has_changed = true
        elseif Bool(sfKeyboard_isKeyPressed(sfKeyW))
            dy -= delta
            has_changed = true
        end

        if Bool(sfKeyboard_isKeyPressed(sfKeyK))
            RTRRT.minkowski_sum_obstacle(tree, 3, 0.01)
        elseif Bool(sfKeyboard_isKeyPressed(sfKeyL))
            RTRRT.minkowski_sum_obstacle(tree, 3, -0.01)
        end
    end
    if has_changed
        RTRRT.update_root_node(tree, dx, dy)
    end
end


# RT RRT* initialization
init_pose = [0.5, 2.0, 0]
params =
    RTRRT.parameters(1.5, [0.0, 10.0], [0.0, 10.0], 2000, 0.4, 0.3, 0.3, 20)

tree = RTRRT.Tree(init_pose, params)
RTRRT.load_default_map(tree.map, params)

RTRRT.expansion(tree)
# RTRRT.rewire(tree)

# Profile.clear()
RTRRT.rewire_from_root(tree)
# Juno.profiler(; C = true)

# SFML initialization[2.0, 3.5]
mode = sfVideoMode(1280, 720, 32)

window = sfRenderWindow_create(mode, "RT-RRT*", sfResize | sfClose, C_NULL)

# Root node marker
root_node_marker = sfCircleShape_create()
radius_coords = 0.05
radius_pixel =
    Int(round((radius_coords / (params.x_range[2] - params.x_range[1])) * 1280))
sfCircleShape_setRadius(root_node_marker, radius_pixel)
sfCircleShape_setPosition(
    root_node_marker,
    CoordsToPixel(
        tree.all_nodes[1].pose[1:2] - [radius_coords, radius_coords],
        params,
    ),
)
sfCircleShape_setFillColor(root_node_marker, sfColor_fromRGB(100, 33, 77))

# Tree edges
edges = sfVertexArray_create()
sfVertexArray_setPrimitiveType(edges, sfLines)

# Goal path
goal_path = sfVertexArray_create()
sfVertexArray_setPrimitiveType(goal_path, sfLines)

# Obstacles
obstacle_shapes = []
for obst in tree.map.all_obstacles
    if length(obst.points) == 2
        points = []
        if obst.p1[1] < obst.p2[1]
            p1 = obst.p1
            p2 = obst.p2
        else
            p1 = obst.p2
            p2 = obst.p1
        end
        d = (p2 - p1) / norm(p2 - p1)
        delta = 0.02
        S(x) = [cos(x) sin(x); -sin(x) cos(x)]
        push!(points, p1 - delta * S(pi / 2) * d)
        push!(points, p1 + delta * S(pi / 2) * d)
        push!(points, p2 + delta * S(pi / 2) * d)
        push!(points, p2 - delta * S(pi / 2) * d)
    else
        points = obst.points
    end
    push!(obstacle_shapes, sfConvexShape_create())
    sfConvexShape_setPointCount(obstacle_shapes[end], length(points))
    for (i, point) in enumerate(points)
        sfConvexShape_setPoint(
            obstacle_shapes[end],
            i - 1,
            CoordsToPixel(point, params),
        )
    end
    sfConvexShape_setFillColor(
        obstacle_shapes[end],
        sfColor_fromRGB(66, 135, 245),
    )
    sfConvexShape_setOutlineColor(
        obstacle_shapes[end],
        sfColor_fromRGB(5, 96, 242),
    )
    sfConvexShape_setOutlineThickness(obstacle_shapes[end], 2)
end



sfRenderWindow_clear(window, sfColor_fromRGBA(255, 255, 255, 1))


event_ref = Ref{sfEvent}()

global i = 1
while Bool(sfRenderWindow_isOpen(window))
    # process events
    while Bool(sfRenderWindow_pollEvent(window, event_ref))
        # close window : exit
        event_ref.x.type == sfEvtClosed && sfRenderWindow_close(window)
    end
    process_events(tree)
    RTRRT.rewire_from_root(tree)

    sfCircleShape_setPosition(
        root_node_marker,
        CoordsToPixel(
            tree.all_nodes[1].pose[1:2] - [radius_coords, radius_coords],
            params,
        ),
    )

    # Obstacles
    empty!(obstacle_shapes)
    for obst in tree.map.all_obstacles
        if length(obst.points) == 2
            points = []
            if obst.p1[1] < obst.p2[1]
                p1 = obst.p1
                p2 = obst.p2
            else
                p1 = obst.p2
                p2 = obst.p1
            end
            d = (p2 - p1) / norm(p2 - p1)
            delta = 0.02
            S(x) = [cos(x) sin(x); -sin(x) cos(x)]
            push!(points, p1 - delta * S(pi / 2) * d)
            push!(points, p1 + delta * S(pi / 2) * d)
            push!(points, p2 + delta * S(pi / 2) * d)
            push!(points, p2 - delta * S(pi / 2) * d)
        else
            points = obst.points
        end
        push!(obstacle_shapes, sfConvexShape_create())
        sfConvexShape_setPointCount(obstacle_shapes[end], length(points))
        for (i, point) in enumerate(points)
            sfConvexShape_setPoint(
                obstacle_shapes[end],
                i - 1,
                CoordsToPixel(point, params),
            )
        end
        sfConvexShape_setFillColor(
            obstacle_shapes[end],
            sfColor_fromRGB(66, 135, 245),
        )
        sfConvexShape_setOutlineColor(
            obstacle_shapes[end],
            sfColor_fromRGB(5, 96, 242),
        )
        sfConvexShape_setOutlineThickness(obstacle_shapes[end], 2)
    end


    # println("ITERATION $i")
    sfRenderWindow_clear(window, sfColor_fromRGBA(255, 255, 255, 1))
    for obst in obstacle_shapes
        sfRenderWindow_drawConvexShape(window, obst, C_NULL)
    end
    update_edges(edges, tree)

    # Get mouse position
    mouse_pos = sfMouse_getPosition(window)
    tree.current_goal = PixelToCoords(mouse_pos, tree.params)
    path_circles = update_goal_path(goal_path, tree)

    sfRenderWindow_drawVertexArray(window, edges, C_NULL)
    sfRenderWindow_drawVertexArray(window, goal_path, C_NULL)
    for circ in path_circles
        sfRenderWindow_drawCircleShape(window, circ, C_NULL)
    end
    sfRenderWindow_drawCircleShape(window, root_node_marker, C_NULL)

    sfRenderWindow_display(window)
    global i += 1
    for circ in path_circles
        sfCircleShape_destroy(circ)
    end

    for obst in obstacle_shapes
        sfConvexShape_destroy(obst)
    end
end


sfVertexArray_destroy(edges)
sfVertexArray_destroy(goal_path)
sfCircleShape_destroy(root_node_marker)
sfRenderWindow_destroy(window)
