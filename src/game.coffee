class Playtime.Game

  constructor: (targetCanvas) ->
    @stage = new createjs.Stage(targetCanvas)
    createjs.Ticker.addListener(@update, true)

  start: =>
    @createShapes()
    @stage.addChild(@circle)
    @stage.addChild(@text)

  createShapes: =>
    @text = new createjs.Text("Some Text", Playtime.Fonts.sansLarge, Playtime.Colors.green)
    @text.x = 10
    @text.y = 20

    @circle = new Playtime.Skins.Circle()
    @circle.x = 20
    @circle.y = 50

  update: (dt) =>
    @circle.x += (dt / 10)
    @circle.y += (dt / 10)
    @text.scaleX += 0.2
    @text.scaleY += 0.2
    @text.rotation += 10

    @stage.update()

