%plots MeasuredPosition graphs
function SRDgraphic_PlotMeasuredPosition(SimulationOutput, varargin)
Parser = inputParser;
Parser.FunctionName = 'SRDgraphic_PlotMeasuredPosition';
Parser.addOptional('NewFigure', true);
Parser.addOptional('Indeces', 1:size(SimulationOutput.MeasuredPosition, 2));
Parser.parse(varargin{:});

NewFigure = Parser.Results.NewFigure;

if NewFigure
    figure_handle = figure('Color', 'w', 'Name', 'Measured Position');
end

labels = {};
for i = 1:length(Parser.Results.Indeces)
    index = Parser.Results.Indeces(i);
    
    plot(SimulationOutput.Time, SimulationOutput.MeasuredPosition(:, index), ...
        'LineWidth', SRDgraphic_get_LineWidth(i), 'LineStyle', SRDgraphic_get_LineStyle(i), ...
        'Color', SRDgraphic_get_Color(i)); hold on;
    
    labels{i} = ['$$\hat q_', num2str(index), '$$'];
end

grid on; grid minor;
ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
ax.FontName = 'Times New Roman';
ax.FontSize = 18;
xlabel_handle = xlabel('$$t$$, s');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$\hat q_i$$');
ylabel_handle.Interpreter = 'latex';

legend_handle = legend(labels);
legend_handle.Interpreter = 'latex';
end