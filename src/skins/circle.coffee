class Playtime.Skins.Circle extends createjs.Shape

  defaultOptions:
    fillColor: Playtime.Colors.green
    radius: 40
    x: 0
    y: 0

  constructor: (args={}) ->
    super
    _.defaults(args, @defaultOptions)
    _.each args, (value, key) => @[key] = args[key]

    @graphics.beginFill(@fillColor)
    @graphics.drawCircle(@x, @y, @radius)

