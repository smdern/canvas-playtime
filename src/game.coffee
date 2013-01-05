class Game

  constructor: (targetCanvas) ->
    @stage = new createjs.Stage(targetCanvas)
    createjs.Ticker.addListener(@update, true)

  start: =>
    @createShapes()
    @stage.addChild(@circle)
    @stage.addChild(@text)

  createShapes: =>
    @text = new createjs.Text("Some Text")
    @text.x = 10
    @text.y = 20

    @circle = new Circle()
    @circle.x = 20
    @circle.y = 50

  update: (dt) =>
    console.log 'updating', dt
    @circle.x += (dt / 10)
    @circle.y += (dt / 10)
    @text.scaleX += 0.2
    @text.scaleY += 0.2
    @text.rotation += 10

    @stage.update()

