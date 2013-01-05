class Circle extends createjs.Shape

  defaultOptions:
    fillColor: createjs.Graphics.getRGB(0,255,0)

  constructor: (args) ->
    super
    _.defaults(args, @defaultOptions)
    @graphics.beginFill(@defaultOptions.fillColor)
    @graphics.drawCircle(0,0,40)

