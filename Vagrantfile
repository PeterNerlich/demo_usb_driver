Vagrant.require_version ">= 1.8.0"

Vagrant.configure(2) do |config|

  config.vm.box = "generic/debian10"

  config.vm.provider :libvirt do |libvirt|
    libvirt.redirdev :type => "spicevmc"
    # Bus 001 Device 037: ID 0547:1002 Anchor Chips, Inc. Python2 WDM Encoder
    libvirt.redirfilter :vendor => "0x0547", :product => "0x1002", :allow => "yes"
    libvirt.redirfilter :allow => "no"
  end

  config.vm.provision "ansible" do |ansible|
    ansible.verbose = "v"
    ansible.playbook = "testing.yml"
  end
end
