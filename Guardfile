
guard 'coffeescript', :input => 'src', :output => 'compiled'

guard :jammit, :config_path => 'assets.yml', :output_folder => 'app' do
  watch(%r{^compiled/(.*)\.js$})
end

