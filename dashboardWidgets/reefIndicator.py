from dashboardWidgets.widgetConfig import WidgetConfig


class ReefIndicator(WidgetConfig):

    def __init__(
        self, xPos, yPos, nt4Topic_in
    ):
        WidgetConfig.__init__(self, nt4Topic_in, xPos, yPos)
        self.nominalHeight = 20
        self.nominalWidth = 20
        self.isVisible = True

    def getJSDeclaration(self):
        return f"var widget{self.idx} = new ReefIndicator('widget{self.idx}', '{self.name}')\n"
