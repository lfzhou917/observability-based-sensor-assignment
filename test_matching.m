% test_matching
%A = rand(10,8);

A=[0.5 0.5; 1 1; 1 3];

[val m1 m2]=bipartite_matching(A);

%%
% Construct an undirected graph from an adjacency matrix.
        % View the edge list of the graph, and then plot the graph.
        A = [0 10 20 30; 10 0 2 0; 20 2 0 1; 30 0 1 0]
        G = graph(A)
        G.Edges
        plot(G)
        
        %%
        % Construct a graph using a list of the end nodes of each edge.
        % Also specify the weight of each edge and the name of each node.
        % View the Edges and Nodes tables of the graph, and then plot
        % G with the edge weights labeled.
        s = [1 1 2 2 3 3];
        t = [4 5 4 5 4 5];
        weights = [1 2 1 6 1 1];
        names = {'r1' 'r2' 'r3' 't1' 't2'};
        G = graph(s,t,weights,names);
        G.Edges
        G.Nodes
        h=plot(G,'EdgeLabel',G.Edges.Weight);
        h.XData(1:3) = 1;
        h.XData(4:end) = 2;
        h.YData(1:3) = linspace(0,1,3);
        h.YData(4:end) = linspace(0,1,2);
        %%
            % Construct the same graph as in the previous example using two 
        % tables to specify edge and node properties.
        s = [1 1 1 2 2 3 3 4 5 5 6 7]';
        t = [2 4 8 3 7 4 6 5 6 8 7 8]';
        weights = [10 10 1 10 1 10 1 1 12 12 12 12]';
        names = {'A' 'B' 'C' 'D' 'E' 'F' 'G' 'H'}';
        EdgeTable = table([s t],weights,'VariableNames',{'EndNodes' 'Weight'})
        NodeTable = table(names,'VariableNames',{'Name'})
        G = graph(EdgeTable,NodeTable)
        plot(G,'EdgeLabel',G.Edges.Weight)
%%
% Make a random MxN adjacency matrix
m = 3;
n = 5;
a = rand(m,n)>.25;
% Expand out to symmetric (M+N)x(M+N) matrix
big_a = [zeros(m,m), a;
         a', zeros(n,n)];     
g = graph(big_a);
% Plot
h = plot(g);
%Make it pretty
h.XData(1:m) = 1;
h.XData((m+1):end) = 2;
h.YData(1:m) = linspace(0,1,m);
h.YData((m+1):end) = linspace(0,1,n);
